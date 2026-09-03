#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        default = str(Path(get_package_share_directory('quadcopter_sim')) / 'config' / 'mission.yaml')
        
        # usawienie parametrów, które zdefinują jak mamy latać
        self.declare_parameter('mission_file', default)
        self.declare_parameter('loop', False)
        self.declare_parameter('start_delay', 2.0)
        
        with open(self.get_parameter('mission_file').value, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        
        self.points = data['waypoints']

        # Sprawdzamy, czy wczytano co najmniej dwa punkty trasy
        if len(self.points) < 2:
            raise ValueError('Mission requires at least two waypoints')
        
        #podane jest informacja czy będziemy latali w kółko i z jakim poóxnieniem zaczyna się process
        self.loop = self.get_parameter('loop').value
        self.delay = float(self.get_parameter('start_delay').value)
        #Rozpoczecie liczenia czasu, za[isywany jest moment czasu]
        self.start = self.get_clock().now()
        # Tworzenie publisherów do publikowania pozycji docelowej, ścieżki referencyjnej i aktualnej ścieżki
        self.pub = self.create_publisher(PoseStamped, 'command/pose', 10)
        # Scieżka referencyjna
        self.ref_pub = self.create_publisher(PathMsg, 'path/reference', 1)
        # Scieżka aktualna
        self.actual_pub = self.create_publisher(PathMsg, 'path/actual', 1)
        self.actual = PathMsg()
        self.actual.header.frame_id = 'world'
        self.odom_samples = 0
        
        # Informujemy o tym, że subskrybujemy wiadomości typu Odometry na temat aktualnej pozycji drona
        self.create_subscription(Odometry, 'odom', self.odom_cb, 20)

        # Tworzymy timer, kótry co 0.02 sekundy wywołuje funkcję tick()
        self.timer = self.create_timer(0.02, self.tick)
        # Odświeżamy kompletną trasę referencyjną, aby RViz uruchomiony nieco
        # później również ją otrzymał i mógł porównać ją ze śladem rzeczywistym.
        self.reference_timer = self.create_timer(1.0, self.publish_reference)
        
        # Czyli self.publish_reference() jest wywoływana w momencie inicjalizacji węzła, aby opublikować początkową 
        # ścieżkę referencyjną na temat misji.
        self.publish_reference()
        
        # Publikujemy liczbę waypointów w misji, aby użytkownik wiedział ile punktów zostało wczytanych.
        self.get_logger().info(f'Loaded mission with {len(self.points)} waypoints')



    @staticmethod
    def pose(x, y, z, yaw, stamp):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = stamp
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        return msg

    def publish_reference(self):
        # Tworzymy pustą listę
        path = PathMsg()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        # Każdy element listy self.points jest słownikiem zawierającym współrzędne x, y, z 
        # oraz opcjonalnie yaw i czas t. 
        # Tworzymy obiekt PoseStamped dla każdego punktu i dodajemy go do listy poses w obiekcie PathMsg.
        # W tym przypadku path/reference opisuje geometrię kompletnej zaplanowanej scieżki. Znacznik czasu oznacza razczej
        # ze cała scieżka została utworzona, opublikowana we chwili"
        for p in self.points:
            path.poses.append(self.pose(p['x'], p['y'], p['z'], p.get('yaw', 0.0), path.header.stamp))
        self.ref_pub.publish(path)

    
    # Oblicza ona aktualny czas od rozpoczęcia misji, uwzględniając opóźnienie startowe.
    #  Następnie interpoluje pozycję drona między dwoma punktami trasy na podstawie upływającego czasu i publikuje 
    # docelową pozycję drona w formacie PoseStamped.
    def tick(self):
        # Obliczamy czas, który upłynął od momentu rozpoczęcia misji, odejmując czas opóźnienia startowego.
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9 - self.delay
        
        # Nie pozwala aby czas był ujemny
        elapsed = max(0.0, elapsed)
        
        # Odczyt długości misji(czas)
        total = float(self.points[-1]['t'])
        
        # Po zakonczeniu trasa rusza od początku
        if self.loop and total > 0.0:
            elapsed %= total
        
        elapsed = min(elapsed, total)
        
        # Czas jest większy bądz równy aktualnemu czasowi misji
        b = next((i for i in range(1, len(self.points)) if self.points[i]['t'] >= elapsed), len(self.points)-1)
        
        # Poprzedni waypoint
        a = max(0, b-1)
        
        # Na podstawie indeksów dobieramy jakie to będą waypointy
        p0, p1 = self.points[a], self.points[b]
        
        # Ile czasu ma dron na przebycie z punktu do punktu
        duration = float(p1['t'] - p0['t'])
        
        
        # u - to jest znormalizowany postęp między waypointami
        # u = 0 oznacza pocżatke
        # u = 1 oznacza koniec, ułamki pomiedzy oznaczają częśc pokonanej drogi
        u = 1.0 if duration <= 0.0 else (elapsed - p0['t']) / duration
        
        # Dzieki temu wzyanczona jest funkcja smoothstep, która zapewnia płynne przejście między punktami trasy.
        smooth = u*u*(3.0-2.0*u)

        # W tej linijce definiowana jest funkcj, jej użycie będzie dopiero niżej przy self.pub.publish
        value = lambda key: p0[key] + smooth * (p1[key]-p0[key])

        
        # tutaj to samo, natomiast dla yaw, ale z uwzględnieniem faktu, że kąt może się zawijać wokół 2π.
        yaw0, yaw1 = p0.get('yaw', 0.0), p1.get('yaw', 0.0)
        yaw = yaw0 + smooth * math.atan2(math.sin(yaw1-yaw0), math.cos(yaw1-yaw0))
        
        # Publikujemy
        self.pub.publish(self.pose(value('x'), value('y'), value('z'), yaw, self.get_clock().now().to_msg()))

    
    # Ta funkcja odbiera bieżącą odometrię drona, zapisuje jego pozycje w historii, a następnie co
    # piąty pomiat publikuje rzeczywistą ścieżkę drona w postaci wiadomości PathMsg. W ten sposób można śledzić, jak dron porusza się w czasie rzeczywistym.
    def odom_cb(self, msg):
        self.odom_samples += 1
        # Zachowujemy 100 punktow na sekunde zamiast kazdej probki fizyki 500 Hz.
        if self.odom_samples % 5 != 0:
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.actual.header.stamp = msg.header.stamp
        self.actual.poses.append(pose)
        if len(self.actual.poses) > 10000:
            self.actual.poses = self.actual.poses[-10000:]
        # Publikacja 10 Hz wystarcza do plynnej wizualizacji i nie przeciaza DDS/RViz.
        if self.odom_samples % 50 == 0:
            self.actual_pub.publish(self.actual)


def main():
    rclpy.init()
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
