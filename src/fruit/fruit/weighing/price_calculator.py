#!/usr/bin/env python3
# src/weighing/price_calculator.py
# 저울에서 측정된 무게와 과일 종류를 기반으로 가격 계산

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
import yaml
import os

class PriceCalculator(Node):
    def __init__(self):
        super().__init__('price_calculator')
        
        # 가격표 로드
        self.declare_parameter('price_table', 'config/price_table.yaml')
        price_table_path = self.get_parameter('price_table').value
        self.load_price_table(price_table_path)
        
        # 현재 저울 위의 과일 정보
        self.current_weight = 0.0  # kg
        self.current_fruit = None
        self.weight_stable = False
        self.last_weight = 0.0
        self.stability_count = 0
        self.stability_threshold = 10  # 10번 연속 안정적이면 확정
        
        # 구독자
        self.weight_sub = self.create_subscription(
            Float32,
            '/scale/weight',
            self.weight_callback,
            10
        )
        
        self.fruit_sub = self.create_subscription(
            String,
            '/current_fruit',
            self.fruit_callback,
            10
        )
        
        # 퍼블리셔
        self.price_pub = self.create_publisher(String, '/fruit_price_info', 10)
        
        # 타이머: 가격 계산 및 publish (1Hz)
        self.timer = self.create_timer(1.0, self.calculate_and_publish)
        
        self.get_logger().info('Price Calculator initialized.')
        self.get_logger().info(f'Loaded prices: {self.prices}')
        self.get_logger().info(f'Currency: {self.currency}, Unit: {self.unit}')
    
    def load_price_table(self, price_table_path):
        """가격표 YAML 파일 로드"""
        # 절대 경로 처리
        if not os.path.isabs(price_table_path):
            workspace_dir = os.getcwd()
            price_table_path = os.path.join(workspace_dir, price_table_path)
        
        if not os.path.exists(price_table_path):
            self.get_logger().error(f'Price table not found: {price_table_path}')
            self.prices = {'banana': 3000, 'orange': 5000, 'Guava': 8000}
            self.currency = 'KRW'
            self.unit = 'kg'
            self.discount_enabled = False
            return
        
        with open(price_table_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        self.prices = data.get('prices', {})
        self.currency = data.get('currency', 'KRW')
        self.unit = data.get('unit', 'kg')
        
        # 할인 정책
        discount_config = data.get('discount', {})
        self.discount_enabled = discount_config.get('enabled', False)
        self.discount_threshold = discount_config.get('bulk_discount', {}).get('threshold', 5.0)
        self.discount_rate = discount_config.get('bulk_discount', {}).get('rate', 0.1)
    
    def weight_callback(self, msg):
        """저울 무게 수신"""
        new_weight = msg.data
        
        # 무게 안정성 체크 (변화량이 1g 이하면 안정적)
        if abs(new_weight - self.last_weight) < 0.001:
            self.stability_count += 1
        else:
            self.stability_count = 0
        
        # 안정적인 무게로 확정
        if self.stability_count >= self.stability_threshold:
            self.weight_stable = True
            self.current_weight = new_weight
        else:
            self.weight_stable = False
        
        self.last_weight = new_weight
    
    def fruit_callback(self, msg):
        """현재 저울 위 과일 종류 수신"""
        self.current_fruit = msg.data
        self.get_logger().info(f'Fruit on scale: {self.current_fruit}')
    
    def calculate_price(self, fruit_name, weight_kg):
        """가격 계산"""
        if fruit_name not in self.prices:
            self.get_logger().warn(f'Unknown fruit: {fruit_name}')
            return 0.0
        
        unit_price = self.prices[fruit_name]  # 원/kg
        total_price = unit_price * weight_kg
        
        # 할인 적용
        if self.discount_enabled and weight_kg >= self.discount_threshold:
            discount_amount = total_price * self.discount_rate
            total_price -= discount_amount
            self.get_logger().info(f'Bulk discount applied: -{discount_amount:.0f}{self.currency}')
        
        return total_price
    
    def calculate_and_publish(self):
        """가격 계산 및 발행"""
        # 무게가 안정적이고 과일이 있을 때만 계산
        if not self.weight_stable or not self.current_fruit or self.current_weight < 0.01:
            return
        
        # 가격 계산
        price = self.calculate_price(self.current_fruit, self.current_weight)
        
        if price <= 0:
            return
        
        # 메시지 생성 (JSON 형식)
        import json
        price_info = {
            'fruit': self.current_fruit,
            'weight': round(self.current_weight, 3),
            'unit': self.unit,
            'unit_price': self.prices.get(self.current_fruit, 0),
            'total_price': round(price, 0),
            'currency': self.currency
        }
        
        msg = String()
        msg.data = json.dumps(price_info, ensure_ascii=False)
        self.price_pub.publish(msg)
        
        # 콘솔 출력
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'🍎 Fruit: {self.current_fruit}')
        self.get_logger().info(f'⚖️  Weight: {self.current_weight:.3f} {self.unit}')
        self.get_logger().info(f'💰 Unit Price: {self.prices[self.current_fruit]:,} {self.currency}/{self.unit}')
        self.get_logger().info(f'💵 Total Price: {price:,.0f} {self.currency}')
        self.get_logger().info('=' * 50)
    
    def get_price_for_fruit(self, fruit_name):
        """외부에서 특정 과일 가격 조회 (Service로 제공 가능)"""
        return self.prices.get(fruit_name, 0)

def main(args=None):
    rclpy.init(args=args)
    node = PriceCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Price Calculator shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

