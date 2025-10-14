#!/usr/bin/env python3
"""
WebSocket 기반 카메라 스트리머 (더 안정적)
- 양방향 통신
- 자동 재연결
- 낮은 지연시간
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import asyncio
import websockets
import base64
import json
from threading import Thread, Lock
import time


class CameraStreamerWebSocket(Node):
    def __init__(self):
        super().__init__('camera_streamer_websocket')
        
        # 파라미터 선언
        self.declare_parameter('websocket_server_url', 'ws://192.168.1.100:8765')
        self.declare_parameter('streaming_fps', 15)
        self.declare_parameter('image_quality', 85)
        self.declare_parameter('image_topic', '/pi_camera/image_raw')
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('max_reconnect_attempts', 0)  # 0 = 무한
        
        # 파라미터 가져오기
        self.ws_url = self.get_parameter('websocket_server_url').value
        self.fps = self.get_parameter('streaming_fps').value
        self.quality = self.get_parameter('image_quality').value
        self.image_topic = self.get_parameter('image_topic').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 최신 이미지 저장
        self.latest_image = None
        self.image_lock = Lock()
        
        # 연결 상태
        self.is_connected = False
        self.reconnect_count = 0
        
        # 구독자 생성
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        
        # WebSocket 이벤트 루프
        self.loop = asyncio.new_event_loop()
        self.ws_thread = Thread(target=self.start_websocket_loop, daemon=True)
        self.ws_thread.start()
        
        # 통계
        self.frames_sent = 0
        self.frames_failed = 0
        self.start_time = time.time()
        
        # 통계 타이머
        self.stats_timer = self.create_timer(10.0, self.print_stats)
        
        self.get_logger().info('='*60)
        self.get_logger().info('WebSocket 카메라 스트리머 시작')
        self.get_logger().info(f'WebSocket URL: {self.ws_url}')
        self.get_logger().info(f'전송 FPS: {self.fps}')
        self.get_logger().info(f'이미지 품질: {self.quality}%')
        self.get_logger().info(f'카메라 토픽: {self.image_topic}')
        self.get_logger().info('='*60)
        
    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.latest_image = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f'이미지 변환 오류: {str(e)}')
    
    def start_websocket_loop(self):
        """WebSocket 이벤트 루프 시작"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.websocket_handler())
    
    async def websocket_handler(self):
        """WebSocket 연결 및 재연결 핸들러"""
        while rclpy.ok():
            try:
                async with websockets.connect(
                    self.ws_url,
                    ping_interval=20,
                    ping_timeout=10,
                    close_timeout=10
                ) as websocket:
                    self.is_connected = True
                    self.reconnect_count = 0
                    self.get_logger().info(f'✓ WebSocket 연결 성공: {self.ws_url}')
                    
                    await self.stream_loop(websocket)
                    
            except websockets.exceptions.WebSocketException as e:
                self.is_connected = False
                self.get_logger().warn(f'WebSocket 오류: {str(e)}')
                await self.handle_reconnect()
                
            except Exception as e:
                self.is_connected = False
                self.get_logger().error(f'예외 발생: {str(e)}')
                await self.handle_reconnect()
    
    async def handle_reconnect(self):
        """재연결 처리"""
        self.reconnect_count += 1
        
        if self.max_reconnect_attempts > 0 and self.reconnect_count >= self.max_reconnect_attempts:
            self.get_logger().error(f'최대 재연결 시도 횟수 초과 ({self.max_reconnect_attempts})')
            return
        
        self.get_logger().info(
            f'재연결 대기 중... ({self.reconnect_count}번째 시도, {self.reconnect_interval}초 후)'
        )
        await asyncio.sleep(self.reconnect_interval)
    
    async def stream_loop(self, websocket):
        """이미지 스트리밍 루프"""
        rate = 1.0 / self.fps
        
        while rclpy.ok() and self.is_connected:
            try:
                with self.image_lock:
                    if self.latest_image is not None:
                        image_copy = self.latest_image.copy()
                    else:
                        await asyncio.sleep(0.1)
                        continue
                
                # JPEG 인코딩
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
                success, buffer = cv2.imencode('.jpg', image_copy, encode_param)
                
                if not success:
                    self.frames_failed += 1
                    continue
                
                # Base64 인코딩
                jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                
                # JSON 메시지 생성
                message = json.dumps({
                    'type': 'image',
                    'data': jpg_as_text,
                    'timestamp': time.time(),
                    'frame_number': self.frames_sent,
                    'width': image_copy.shape[1],
                    'height': image_copy.shape[0]
                })
                
                # WebSocket으로 전송
                await websocket.send(message)
                self.frames_sent += 1
                
                # FPS 조절
                await asyncio.sleep(rate)
                
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().warn('WebSocket 연결이 닫혔습니다')
                break
            except Exception as e:
                self.get_logger().error(f'스트리밍 오류: {str(e)}')
                self.frames_failed += 1
                await asyncio.sleep(0.1)
    
    def print_stats(self):
        """통계 출력"""
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            fps = self.frames_sent / elapsed
            success_rate = (self.frames_sent / (self.frames_sent + self.frames_failed) * 100) if (self.frames_sent + self.frames_failed) > 0 else 0
            
            status = '연결됨' if self.is_connected else '연결 끊김'
            
            self.get_logger().info(
                f'[통계] 상태: {status} | '
                f'전송: {self.frames_sent}프레임 | '
                f'실패: {self.frames_failed} | '
                f'평균 FPS: {fps:.2f} | '
                f'성공률: {success_rate:.1f}%'
            )
    
    def destroy_node(self):
        """노드 종료"""
        self.is_connected = False
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        streamer = CameraStreamerWebSocket()
        rclpy.spin(streamer)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

