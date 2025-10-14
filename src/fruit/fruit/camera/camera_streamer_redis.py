#!/usr/bin/env python3
"""
Redis Pub/Sub 기반 카메라 스트리머
- 초고속 in-memory 메시징
- Pub/Sub 패턴
- 여러 구독자 지원
- 간단한 설정
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import redis
import json
import base64
import time
from threading import Lock


class CameraStreamerRedis(Node):
    def __init__(self):
        super().__init__('camera_streamer_redis')
        
        # 파라미터 선언
        self.declare_parameter('redis_host', '192.168.1.100')
        self.declare_parameter('redis_port', 6379)
        self.declare_parameter('redis_db', 0)
        self.declare_parameter('redis_password', '')
        self.declare_parameter('channel_name', 'camera_stream')
        self.declare_parameter('streaming_fps', 30)
        self.declare_parameter('image_quality', 85)
        self.declare_parameter('image_topic', '/pi_camera/image_raw')
        self.declare_parameter('max_message_size', 10485760)  # 10MB
        
        # 파라미터 가져오기
        self.redis_host = self.get_parameter('redis_host').value
        self.redis_port = self.get_parameter('redis_port').value
        self.redis_db = self.get_parameter('redis_db').value
        self.redis_password = self.get_parameter('redis_password').value
        self.channel_name = self.get_parameter('channel_name').value
        self.fps = self.get_parameter('streaming_fps').value
        self.quality = self.get_parameter('image_quality').value
        self.image_topic = self.get_parameter('image_topic').value
        self.max_message_size = self.get_parameter('max_message_size').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Redis 클라이언트 연결
        try:
            self.redis_client = redis.Redis(
                host=self.redis_host,
                port=self.redis_port,
                db=self.redis_db,
                password=self.redis_password if self.redis_password else None,
                decode_responses=False,  # 바이너리 데이터 전송
                socket_connect_timeout=5,
                socket_keepalive=True
            )
            # 연결 테스트
            self.redis_client.ping()
            self.get_logger().info(f'✓ Redis 연결 성공: {self.redis_host}:{self.redis_port}')
        except redis.ConnectionError as e:
            self.get_logger().error(f'✗ Redis 연결 실패: {str(e)}')
            raise
        
        # 최신 이미지
        self.latest_image = None
        self.image_lock = Lock()
        
        # 구독자
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        
        # 전송 타이머
        self.stream_timer = self.create_timer(1.0 / self.fps, self.stream_callback)
        
        # 통계
        self.frames_sent = 0
        self.frames_failed = 0
        self.total_bytes_sent = 0
        self.start_time = time.time()
        
        # 통계 타이머
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Redis Pub/Sub 카메라 스트리머 시작')
        self.get_logger().info(f'Redis 서버: {self.redis_host}:{self.redis_port}')
        self.get_logger().info(f'채널: {self.channel_name}')
        self.get_logger().info(f'전송 FPS: {self.fps}')
        self.get_logger().info(f'이미지 품질: {self.quality}%')
        self.get_logger().info('='*60)
    
    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.latest_image = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f'이미지 변환 오류: {str(e)}')
    
    def stream_callback(self):
        """Redis로 이미지 전송"""
        with self.image_lock:
            if self.latest_image is None:
                return
            image_copy = self.latest_image.copy()
        
        try:
            # JPEG 인코딩
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            success, buffer = cv2.imencode('.jpg', image_copy, encode_param)
            
            if not success:
                self.frames_failed += 1
                return
            
            # 메시지 크기 체크
            image_bytes = buffer.tobytes()
            if len(image_bytes) > self.max_message_size:
                self.get_logger().warn(
                    f'이미지 크기 초과: {len(image_bytes)} > {self.max_message_size}'
                )
                self.frames_failed += 1
                return
            
            # Base64 인코딩 (선택사항)
            # Redis는 바이너리도 지원하지만, JSON 호환성을 위해 Base64 사용 가능
            
            # 방법 1: 바이너리 직접 전송 (더 효율적)
            message = {
                'timestamp': time.time(),
                'frame_number': self.frames_sent,
                'width': image_copy.shape[1],
                'height': image_copy.shape[0],
                'encoding': 'jpeg',
                'data': image_bytes  # 바이너리 데이터
            }
            
            # JSON으로 직렬화 (바이너리는 별도 필드로)
            metadata = {
                'timestamp': message['timestamp'],
                'frame_number': message['frame_number'],
                'width': message['width'],
                'height': message['height'],
                'encoding': message['encoding'],
                'size': len(image_bytes)
            }
            
            # Redis에 발행
            # 방법 1: 메타데이터와 이미지 데이터를 분리하여 전송
            self.redis_client.publish(
                f'{self.channel_name}:metadata',
                json.dumps(metadata)
            )
            self.redis_client.publish(
                f'{self.channel_name}:data',
                image_bytes
            )
            
            # 방법 2: 하나의 메시지로 전송 (Base64)
            # base64_data = base64.b64encode(image_bytes).decode('utf-8')
            # metadata['data'] = base64_data
            # self.redis_client.publish(self.channel_name, json.dumps(metadata))
            
            self.frames_sent += 1
            self.total_bytes_sent += len(image_bytes)
            
        except redis.RedisError as e:
            self.get_logger().error(f'Redis 오류: {str(e)}')
            self.frames_failed += 1
            
            # 재연결 시도
            try:
                self.redis_client.ping()
            except:
                self.get_logger().warn('Redis 재연결 시도 중...')
                try:
                    self.redis_client = redis.Redis(
                        host=self.redis_host,
                        port=self.redis_port,
                        db=self.redis_db,
                        password=self.redis_password if self.redis_password else None,
                        decode_responses=False
                    )
                    self.redis_client.ping()
                    self.get_logger().info('✓ Redis 재연결 성공')
                except Exception as e:
                    self.get_logger().error(f'재연결 실패: {str(e)}')
        
        except Exception as e:
            self.get_logger().error(f'스트리밍 오류: {str(e)}')
            self.frames_failed += 1
    
    def print_stats(self):
        """통계 출력"""
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            fps = self.frames_sent / elapsed
            success_rate = (
                self.frames_sent / (self.frames_sent + self.frames_failed) * 100
                if (self.frames_sent + self.frames_failed) > 0 else 0
            )
            bandwidth = (self.total_bytes_sent / elapsed) / 1024 / 1024  # MB/s
            
            # Redis 정보
            try:
                info = self.redis_client.info('stats')
                total_connections = info.get('total_connections_received', 0)
                connected_clients = info.get('connected_clients', 0)
                
                self.get_logger().info(
                    f'[통계] 전송: {self.frames_sent} | '
                    f'실패: {self.frames_failed} | '
                    f'FPS: {fps:.2f} | '
                    f'성공률: {success_rate:.1f}% | '
                    f'대역폭: {bandwidth:.2f} MB/s | '
                    f'Redis 클라이언트: {connected_clients}'
                )
            except:
                self.get_logger().info(
                    f'[통계] 전송: {self.frames_sent} | '
                    f'실패: {self.frames_failed} | '
                    f'FPS: {fps:.2f} | '
                    f'성공률: {success_rate:.1f}%'
                )
    
    def destroy_node(self):
        """노드 종료"""
        if self.redis_client:
            self.redis_client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        streamer = CameraStreamerRedis()
        rclpy.spin(streamer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'오류: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

