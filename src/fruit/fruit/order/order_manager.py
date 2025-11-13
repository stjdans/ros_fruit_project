#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
주문 관리 모듈
한 바구니의 과일들을 하나의 주문으로 생성하고 관리합니다.
"""

import sqlite3
import os
from datetime import datetime
from typing import List, Dict, Optional, Tuple


class OrderManager:
    """주문 생성 및 관리 클래스"""
    
    def __init__(self, db_path: str = None):
        """
        주문 관리자 초기화
        
        Args:
            db_path: 데이터베이스 파일 경로 (기본값: src/db/mydb.db)
        """
        if db_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            db_path = os.path.join(current_dir, '../db/mydb.db')
        
        self.db_path = db_path
        self._ensure_db_exists()
    
    def _ensure_db_exists(self):
        """데이터베이스 파일 존재 확인"""
        if not os.path.exists(self.db_path):
            raise FileNotFoundError(f"데이터베이스 파일이 없습니다: {self.db_path}")
    
    def _get_connection(self) -> sqlite3.Connection:
        """데이터베이스 연결 생성"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row  # 딕셔너리 형태로 결과 반환
        return conn
    
    def generate_order_number(self) -> str:
        """
        주문 번호 생성 (ORD-YYYYMMDD-XXXX)
        
        Returns:
            생성된 주문 번호
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        today = datetime.now().strftime('%Y%m%d')
        prefix = f"ORD-{today}-"
        
        # 오늘 날짜의 마지막 주문 번호 조회
        cursor.execute("""
            SELECT order_number FROM orders 
            WHERE order_number LIKE ? 
            ORDER BY order_number DESC 
            LIMIT 1
        """, (f"{prefix}%",))
        
        result = cursor.fetchone()
        conn.close()
        
        if result:
            last_number = int(result['order_number'].split('-')[-1])
            new_number = last_number + 1
        else:
            new_number = 1
        
        return f"{prefix}{new_number:04d}"
    
    def create_order(self, fruits_data: List[Dict], priority: int = 0) -> Tuple[int, str]:
        """
        새 주문 생성 (한 바구니의 모든 과일을 하나의 주문으로 생성)
        
        Args:
            fruits_data: 과일 정보 리스트
                예: [
                    {'fruit_name': 'banana', 'weight': 0.5, 'quantity': 2},
                    {'fruit_name': 'apple', 'weight': 0.3, 'quantity': 3}
                ]
            priority: 주문 우선순위 (기본값: 0)
        
        Returns:
            (order_id, order_number) 튜플
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        try:
            # 주문 번호 생성
            order_number = self.generate_order_number()
            
            # 주문 기본 정보 생성
            cursor.execute("""
                INSERT INTO orders (order_number, status, total_amount, total_weight, priority)
                VALUES (?, 'Pending', 0, 0, ?)
            """, (order_number, priority))
            
            order_id = cursor.lastrowid
            
            # 각 과일별 주문 아이템 추가
            for fruit_data in fruits_data:
                self._add_order_item(cursor, order_id, fruit_data)
            
            # 로그 기록
            self._add_robot_log(
                cursor, order_id, 
                'order_creation', 
                'Success', 
                f"주문 생성 완료: {len(fruits_data)}종류의 과일"
            )
            
            conn.commit()
            return order_id, order_number
            
        except Exception as e:
            conn.rollback()
            raise Exception(f"주문 생성 실패: {str(e)}")
        finally:
            conn.close()
    
    def _add_order_item(self, cursor: sqlite3.Cursor, order_id: int, fruit_data: Dict):
        """
        주문 아이템 추가 (과일 무게 * kg당 가격 계산)
        
        Args:
            cursor: 데이터베이스 커서
            order_id: 주문 ID
            fruit_data: 과일 정보 {'fruit_name': str, 'weight': float, 'quantity': int}
        """
        fruit_name = fruit_data.get('fruit_name') or fruit_data.get('name_en')
        weight = float(fruit_data['weight'])
        quantity = int(fruit_data.get('quantity', 1))
        
        # 과일 정보 조회 (가격 포함)
        cursor.execute("""
            SELECT id, name 
            FROM fruits 
            WHERE name = ?
        """, (fruit_name,))
        
        fruit = cursor.fetchone()
        if not fruit:
            raise ValueError(f"과일을 찾을 수 없거나 판매 중단됨: {fruit_name}")
        
        fruit_id = fruit['id']
        # 실제 DB에는 price_per_kg 컬럼이 없으므로 임시로 고정 가격 사용
        # TODO: 실제 가격 테이블이나 설정에서 가격을 가져와야 함
        unit_price = 5000  # 임시 고정 가격 (원/kg)
        
        # 소계 계산: 무게(kg) * kg당 가격
        subtotal = weight * unit_price
        
        # 주문 아이템 추가
        cursor.execute("""
            INSERT INTO order_items (order_id, fruit_id, quantity, weight, unit_price, subtotal)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (order_id, fruit_id, quantity, weight, unit_price, subtotal))
        
        print(f"  - {fruit['name']}: {weight}kg × {unit_price:,}원/kg = {subtotal:,.0f}원")
    
    def update_order_status(self, order_id: int, status: str, 
                           error_message: str = None, 
                           processing_time: int = None):
        """
        주문 상태 업데이트
        
        Args:
            order_id: 주문 ID
            status: 새 상태 (Pending/Processing/Completed/Failed)
            error_message: 에러 메시지 (실패 시)
            processing_time: 처리 시간(초)
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        try:
            update_fields = ["status = ?"]
            params = [status]
            
            if status == 'Processing':
                update_fields.append("started_at = CURRENT_TIMESTAMP")
            elif status in ['Completed', 'Failed']:
                update_fields.append("completed_at = CURRENT_TIMESTAMP")
            
            if error_message:
                update_fields.append("error_message = ?")
                params.append(error_message)
            
            if processing_time is not None:
                update_fields.append("processing_time = ?")
                params.append(processing_time)
            
            params.append(order_id)
            
            cursor.execute(f"""
                UPDATE orders 
                SET {', '.join(update_fields)}
                WHERE id = ?
            """, params)
            
            # 로그 기록
            log_message = f"주문 상태 변경: {status}"
            if error_message:
                log_message += f" - {error_message}"
            
            self._add_robot_log(
                cursor, order_id,
                'status_update',
                'Success' if status != 'Failed' else 'Failed',
                log_message
            )
            
            conn.commit()
            
        except Exception as e:
            conn.rollback()
            raise Exception(f"주문 상태 업데이트 실패: {str(e)}")
        finally:
            conn.close()
    
    def get_order(self, order_id: int = None, order_number: str = None) -> Optional[Dict]:
        """
        주문 조회
        
        Args:
            order_id: 주문 ID
            order_number: 주문 번호
        
        Returns:
            주문 정보 딕셔너리 (아이템 포함)
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        try:
            # 주문 기본 정보 조회
            if order_id:
                cursor.execute("SELECT * FROM orders WHERE id = ?", (order_id,))
            elif order_number:
                cursor.execute("SELECT * FROM orders WHERE order_number = ?", (order_number,))
            else:
                raise ValueError("order_id 또는 order_number 중 하나는 필수입니다")
            
            order = cursor.fetchone()
            if not order:
                return None
            
            order_dict = dict(order)
            
            # 주문 아이템 조회
            cursor.execute("""
                SELECT 
                    oi.*,
                    f.name as fruit_name
                FROM order_items oi
                JOIN fruits f ON oi.fruit_id = f.id
                WHERE oi.order_id = ?
            """, (order_dict['id'],))
            
            order_dict['items'] = [dict(row) for row in cursor.fetchall()]
            
            return order_dict
            
        finally:
            conn.close()
    
    def get_pending_orders(self) -> List[Dict]:
        """
        대기 중인 주문 조회 (우선순위 높은 순)
        
        Returns:
            주문 리스트
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        try:
            cursor.execute("""
                SELECT * FROM orders 
                WHERE status = 'Pending' 
                ORDER BY priority DESC, created_at ASC
            """)
            
            return [dict(row) for row in cursor.fetchall()]
            
        finally:
            conn.close()
    
    def _add_robot_log(self, cursor: sqlite3.Cursor, order_id: int, 
                       step: str, status: str, message: str = None, 
                       duration: float = None):
        """
        로봇 작업 로그 추가
        
        Args:
            cursor: 데이터베이스 커서
            order_id: 주문 ID
            step: 작업 단계
            status: 상태 (Success/Failed/InProgress)
            message: 로그 메시지
            duration: 소요 시간(초)
        """
        cursor.execute("""
            INSERT INTO robot_logs (order_id, step, status, message, duration)
            VALUES (?, ?, ?, ?, ?)
        """, (order_id, step, status, message, duration))
    
    def add_robot_log(self, order_id: int, step: str, status: str, 
                     message: str = None, duration: float = None):
        """
        로봇 작업 로그 추가 (외부 호출용)
        
        Args:
            order_id: 주문 ID
            step: 작업 단계 (fruit_detection, weight_measurement, picking, placing 등)
            status: 상태 (Success/Failed/InProgress)
            message: 로그 메시지
            duration: 소요 시간(초)
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        try:
            self._add_robot_log(cursor, order_id, step, status, message, duration)
            conn.commit()
        finally:
            conn.close()
    
    def get_order_logs(self, order_id: int) -> List[Dict]:
        """
        주문의 로봇 작업 로그 조회
        
        Args:
            order_id: 주문 ID
        
        Returns:
            로그 리스트
        """
        conn = self._get_connection()
        cursor = conn.cursor()
        
        try:
            cursor.execute("""
                SELECT * FROM robot_logs 
                WHERE order_id = ? 
                ORDER BY created_at ASC
            """, (order_id,))
            
            return [dict(row) for row in cursor.fetchall()]
            
        finally:
            conn.close()
    
    def get_fruit_price(self, fruit_name: str) -> Optional[float]:
        """
        과일의 현재 가격 조회
        
        Args:
            fruit_name: 과일명
        
        Returns:
            kg당 가격 (현재는 고정 가격 반환)
        """
        # 실제 DB에는 price_per_kg 컬럼이 없으므로 임시로 고정 가격 반환
        # TODO: 실제 가격 테이블이나 설정에서 가격을 가져와야 함
        return 5000.0  # 임시 고정 가격 (원/kg)


def main():
    """테스트 코드"""
    print("=" * 60)
    print("주문 관리자 테스트")
    print("=" * 60)
    
    # OrderManager 인스턴스 생성
    manager = OrderManager()
    
    # 테스트: 한 바구니에 여러 과일이 담긴 주문 생성
    print("\n[테스트 1] 바구니 주문 생성")
    print("-" * 60)
    
    basket_fruits = [
        {'fruit_name': '바나나', 'weight': 0.5, 'quantity': 2},   # 바나나 0.5kg
        {'fruit_name': '사과', 'weight': 0.3, 'quantity': 3},    # 사과 0.3kg
        {'fruit_name': '오렌지', 'weight': 0.4, 'quantity': 2}    # 오렌지 0.4kg
    ]
    
    try:
        order_id, order_number = manager.create_order(basket_fruits, priority=1)
        print(f"\n✅ 주문 생성 성공!")
        print(f"   주문 ID: {order_id}")
        print(f"   주문 번호: {order_number}")
        
        # 주문 조회
        print(f"\n[테스트 2] 주문 조회")
        print("-" * 60)
        order = manager.get_order(order_id=order_id)
        
        print(f"주문 번호: {order['order_number']}")
        print(f"상태: {order['status']}")
        print(f"총 무게: {order['total_weight']}kg")
        print(f"총 금액: {order['total_amount']:,.0f}원")
        print(f"\n주문 아이템:")
        
        for item in order['items']:
            print(f"  - {item['fruit_name']}")
            print(f"    수량: {item['quantity']}개, 무게: {item['weight']}kg")
            print(f"    단가: {item['unit_price']:,.0f}원/kg")
            print(f"    소계: {item['subtotal']:,.0f}원")
        
        # 주문 상태 업데이트
        print(f"\n[테스트 3] 주문 상태 업데이트")
        print("-" * 60)
        manager.update_order_status(order_id, 'Processing')
        print("✅ 주문 상태를 'Processing'으로 변경")
        
        manager.add_robot_log(order_id, 'picking', 'InProgress', '과일 픽업 시작')
        print("✅ 로봇 로그 추가")
        
        # 대기 중인 주문 조회
        print(f"\n[테스트 4] 대기 중인 주문 조회")
        print("-" * 60)
        pending = manager.get_pending_orders()
        print(f"대기 중인 주문: {len(pending)}건")
        
    except Exception as e:
        print(f"❌ 에러 발생: {str(e)}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()

