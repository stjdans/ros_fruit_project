#!/usr/bin/env python3
"""
ROS 2 Parameter Web Server
웹 브라우저에서 ROS 2 파라미터를 제어할 수 있는 REST API 서버
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading


app = Flask(__name__)
CORS(app)  # CORS 허용
host = '0.0.0.0'
port = 5002
debug = True

class ParameterServer(Node):
    def __init__(self):
        super().__init__('parameter_server')
        self.get_logger().info('Parameter Web Server 시작')
        
        # 클라이언트 캐시 (노드별로 재사용)
        self.set_param_clients = {}
        self.get_param_clients = {}
    
    def set_node_parameter(self, node_name, param_name, param_value):
        """다른 노드의 파라미터 설정"""
        self.get_logger().info(f'set_node_parameter: {node_name}, {param_name}, {param_value}')
        try:
            # 캐시에서 클라이언트 가져오기 또는 생성
            if node_name not in self.set_param_clients:
                self.set_param_clients[node_name] = self.create_client(
                    SetParameters,
                    f'{node_name}/set_parameters'
                )
                self.get_logger().info(f'새 SET 클라이언트 생성: {node_name}')
            
            param_client = self.set_param_clients[node_name]
            
            # 서비스가 준비될 때까지 대기
            if not param_client.wait_for_service(timeout_sec=2.0):
                return False, "Node not found"
            
            # 파라미터 타입 결정
            if isinstance(param_value, bool):
                param = Parameter(param_name, Parameter.Type.BOOL, param_value)
            elif isinstance(param_value, int):
                param = Parameter(param_name, Parameter.Type.INTEGER, param_value)
            elif isinstance(param_value, float):
                param = Parameter(param_name, Parameter.Type.DOUBLE, param_value)
            elif isinstance(param_value, str):
                param = Parameter(param_name, Parameter.Type.STRING, param_value)
            else:
                return False, "Unsupported parameter type"
            
            # 파라미터 설정 요청
            request = SetParameters.Request()
            request.parameters = [param.to_parameter_msg()]
            
            future = param_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                return True, "Parameter set successfully"
            else:
                return False, "Failed to set parameter"
                
        except Exception as e:
            return False, str(e)
    
    def get_node_parameter(self, node_name, param_name):
        """다른 노드의 파라미터 읽기"""
        try:
            # 캐시에서 클라이언트 가져오기 또는 생성
            if node_name not in self.get_param_clients:
                self.get_param_clients[node_name] = self.create_client(
                    GetParameters,
                    f'{node_name}/get_parameters'
                )
                self.get_logger().info(f'새 GET 클라이언트 생성: {node_name}')
            
            param_client = self.get_param_clients[node_name]
            
            if not param_client.wait_for_service(timeout_sec=2.0):
                return None, "Node not found"
            
            request = GetParameters.Request()
            request.names = [param_name]
            
            future = param_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None and len(future.result().values) > 0:
                param_value = future.result().values[0]
                # 타입에 따라 값 추출
                if param_value.type == 1:  # BOOL
                    return param_value.bool_value, None
                elif param_value.type == 2:  # INTEGER
                    return param_value.integer_value, None
                elif param_value.type == 3:  # DOUBLE
                    return param_value.double_value, None
                elif param_value.type == 4:  # STRING
                    return param_value.string_value, None
            
            return None, "Parameter not found"
            
        except Exception as e:
            return None, str(e)
    
    def destroy_node(self):
        """노드 종료 시 모든 클라이언트 정리"""
        self.get_logger().info('클라이언트 정리 중...')
        
        # SET 클라이언트 정리
        for node_name, client in self.set_param_clients.items():
            self.destroy_client(client)
            self.get_logger().info(f'SET 클라이언트 삭제: {node_name}')
        
        # GET 클라이언트 정리
        for node_name, client in self.get_param_clients.items():
            self.destroy_client(client)
            self.get_logger().info(f'GET 클라이언트 삭제: {node_name}')
        
        self.set_param_clients.clear()
        self.get_param_clients.clear()
        
        super().destroy_node()

# ROS 노드 인스턴스
param_server = None

@app.route('/api/parameter/set', methods=['POST'])
def set_parameter():
    """파라미터 설정 API"""
    data = request.json
    node_name = data.get('node_name', '/camera_streamer_zeromq')
    param_name = data.get('param_name')
    param_value = data.get('param_value')
    
    if not param_name:
        return jsonify({'success': False, 'message': 'param_name required'}), 400
    
    success, message = param_server.set_node_parameter(node_name, param_name, param_value)
    
    return jsonify({
        'success': success,
        'message': message,
        'node_name': node_name,
        'param_name': param_name,
        'param_value': param_value
    })

@app.route('/api/parameter/get', methods=['GET'])
def get_parameter():
    """파라미터 읽기 API"""
    node_name = request.args.get('node_name', '/camera_streamer_zeromq')
    param_name = request.args.get('param_name')
    
    if not param_name:
        return jsonify({'success': False, 'message': 'param_name required'}), 400
    
    value, error = param_server.get_node_parameter(node_name, param_name)
    
    if error:
        return jsonify({'success': False, 'message': error}), 404
    
    return jsonify({
        'success': True,
        'node_name': node_name,
        'param_name': param_name,
        'param_value': value
    })

@app.route('/api/yolo/toggle', methods=['POST'])
def toggle_yolo():
    """YOLO ON/OFF 토글"""
    data = request.json
    use_yolo = data.get('use_yolo', True)
    
    success, message = param_server.set_node_parameter(
        '/camera_streamer_zeromq',
        'use_yolo',
        use_yolo
    )
    
    return jsonify({
        'success': success,
        'message': message,
        'use_yolo': use_yolo
    })

@app.route('/api/status', methods=['GET'])
def status():
    """서버 상태 확인"""
    return jsonify({
        'success': True,
        'message': 'ROS2 Parameter Server is running',
        'ros_ok': rclpy.ok()
    })

def run_flask():
    """Flask 서버 실행 (별도 스레드)"""
    # debug=False, use_reloader=False: 스레드 안전성
    app.run(host=host, port=port, debug=debug, use_reloader=False)

def main(args=None):
    global param_server
    
    rclpy.init(args=args)
    param_server = ParameterServer()
    
    # Flask 서버를 별도 스레드에서 실행
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    param_server.get_logger().info('='*60)
    param_server.get_logger().info('Parameter Web Server 실행 중')
    param_server.get_logger().info(f'REST API: http://{host}:{port}')
    param_server.get_logger().info('='*60)
    
    try:
        rclpy.spin(param_server)
    except KeyboardInterrupt:
        pass
    finally:
        param_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

