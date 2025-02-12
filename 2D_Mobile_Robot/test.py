import tkinter as tk
from tkinter import scrolledtext
import random
import math
import heapq
import threading
import time

class VirtualRoboticsLab(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Virtual Robotics Lab")
        self.geometry("800x600")
        
        # 그리드 크기 설정
        self.grid_size = 5  # 숫자 값으로 정의
        self.robot_radius = 15  # 로봇의 충돌 범위 설정 (기존 크기보다 넓게 설정)
        self.inflation_radius = 3  # 장애물 인플레이션 크기 설정
        
        # 중앙 시뮬레이션 패널
        frame_simulation = tk.Frame(self, width=600, height=600, bg="white")
        frame_simulation.grid(row=0, column=0, rowspan=2, padx=10, pady=10, sticky="nsew")
        label_simulation = tk.Label(frame_simulation, text="2D Simulation", bg="white")
        label_simulation.grid(row=0, column=0)
        self.canvas = tk.Canvas(frame_simulation, width=400, height=400, bg="white")
        self.canvas.grid(row=1, column=0)
        self.walls = []
        self.obstacles = []
        # 그리드 시스템 추가
        self.draw_grid()
        
        # 로봇 초기 위치와 방향 설정 (시작 좌표는 중앙, 방향은 0도)
        self.robot_x = 200
        self.robot_y = 200
        self.robot_angle = 0  # 로봇의 초기 방향 (0도, 오른쪽)
        self.robot_size = 10

        # 로봇 모양 (사각형)
        self.robot = self.canvas.create_rectangle(
            self.robot_x - self.robot_size, self.robot_y - self.robot_size,
            self.robot_x + self.robot_size, self.robot_y + self.robot_size, fill="blue"
        )
        
        # 로봇의 방향을 나타내는 화살표
        self.robot_direction = self.canvas.create_line(self.robot_x, self.robot_y, self.robot_x + 30, self.robot_y, arrow=tk.LAST)
        
        # 로봇의 충돌 범위 표시 (회색 원으로 표현)
        self.robot_collision_range = self.canvas.create_oval(
            self.robot_x - self.robot_radius, self.robot_y - self.robot_radius,
            self.robot_x + self.robot_radius, self.robot_y + self.robot_radius,
            outline="gray", width=2
        )    
        # 목표 지점 (랜덤)
        self.goal = self.place_goal()

        frame_analysis = tk.Frame(self, width=200, height=600, bg="lightgray")
        frame_analysis.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        label_analysis = tk.Label(frame_analysis, text="Performance Analysis", bg="lightgray")
        label_analysis.grid(row=0, column=0)
        self.log_text = scrolledtext.ScrolledText(frame_analysis, width=25, height=30)
        self.log_text.grid(row=1, column=0, padx=5, pady=5)
        
        # 장애물 설정 (랜덤 장애물)
        self.num_obstacles = 5  # 장애물 개수
        self.place_obstacles()
        
        # 벽 설정 (그리드 경계)
        self.walls = set()
        for x in range(0, 400, self.grid_size):
            self.walls.add((x, 0))
            self.walls.add((x, 400))
        for y in range(0, 400, self.grid_size):
            self.walls.add((0, y))
            self.walls.add((400, y))
        
        # 제어 버튼 추가
        self.control_frame = tk.Frame(self, bg="lightgray")
        self.control_frame.grid(row=1, column=0, columnspan=2, pady=10)
        
        self.start_button = tk.Button(self.control_frame, text="Start Pathfinding", command=self.run_pathfinding)
        self.start_button.grid(row=0, column=0, padx=10, pady=5)

        self.forward_button = tk.Button(self.control_frame, text="Move Forward", command=self.move_forward)
        self.forward_button.grid(row=0, column=1, padx=10, pady=5)

        self.backward_button = tk.Button(self.control_frame, text="Move Backward", command=self.move_backward)
        self.backward_button.grid(row=0, column=2, padx=10, pady=5)

        self.rotate_button = tk.Button(self.control_frame, text="Rotate", command=self.rotate_robot)
        self.rotate_button.grid(row=0, column=3, padx=10, pady=5)

        # 새로 고침 버튼 추가
        self.refresh_goal_button = tk.Button(self.control_frame, text="Refresh Goal", command=self.refresh_goal)
        self.refresh_goal_button.grid(row=1, column=0, padx=10, pady=5)

        self.refresh_obstacles_button = tk.Button(self.control_frame, text="Refresh Obstacles", command=self.refresh_obstacles)
        self.refresh_obstacles_button.grid(row=1, column=1, padx=10, pady=5)
        
        # 로봇 위치 랜덤화 버튼 추가
        self.randomize_robot_button = tk.Button(self.control_frame, text="Randomize Robot Position", command=self.randomize_robot_position)
        self.randomize_robot_button.grid(row=1, column=2, padx=10, pady=5)
        
        # 추가: 입력창 및 버튼 추가
        self.robot_x_label = tk.Label(self.control_frame, text="Robot X:")
        self.robot_x_label.grid(row=2, column=0)
        self.robot_x_entry = tk.Entry(self.control_frame)
        self.robot_x_entry.grid(row=2, column=1)

        self.robot_y_label = tk.Label(self.control_frame, text="Robot Y:")
        self.robot_y_label.grid(row=2, column=2)
        self.robot_y_entry = tk.Entry(self.control_frame)
        self.robot_y_entry.grid(row=2, column=3)

        self.set_position_button = tk.Button(self.control_frame, text="Set Robot Position", command=self.set_robot_position)
        self.set_position_button.grid(row=2, column=4)

        self.inflation_radius_label = tk.Label(self.control_frame, text="Inflation Radius:")
        self.inflation_radius_label.grid(row=3, column=0)
        self.inflation_radius_entry = tk.Entry(self.control_frame)
        self.inflation_radius_entry.grid(row=3, column=1)

        self.num_obstacles_label = tk.Label(self.control_frame, text="Num Obstacles:")
        self.num_obstacles_label.grid(row=3, column=2)
        self.num_obstacles_entry = tk.Entry(self.control_frame)
        self.num_obstacles_entry.grid(row=3, column=3)

        self.refresh_obstacles_button = tk.Button(self.control_frame, text="Set Obstacles", command=self.set_obstacles)
        self.refresh_obstacles_button.grid(row=3, column=4)
        
    def draw_grid(self):
        # 그리드를 그리는 함수
        for i in range(0, 400, self.grid_size):
            self.canvas.create_line(i, 0, i, 400, fill="lightgray", dash=(2, 2))
            self.canvas.create_line(0, i, 400, i, fill="lightgray", dash=(2, 2))

    def place_goal(self):
        # 목표 지점을 랜덤으로 배치하고 유효한 위치인지를 확인
        goal_x = random.randint(50, 350)
        goal_y = random.randint(50, 350)
        
        while not self.is_valid((goal_x, goal_y)):  # 목표가 장애물이나 벽에 겹치지 않도록 확인
            goal_x = random.randint(50, 350)
            goal_y = random.randint(50, 350)
        
        # 목표 지점 그리기
        return self.canvas.create_oval(goal_x-5, goal_y-5, goal_x+5, goal_y+5, fill="green")
    
    def place_obstacles(self):
        # 기존 장애물을 제거하고 새로운 장애물을 생성하는 함수
        self.obstacles = []
        for _ in range(self.num_obstacles):
            x1 = random.randint(50, 350)
            y1 = random.randint(50, 350)
            x2 = x1 + random.randint(20, 50)
            y2 = y1 + random.randint(20, 50)
            inflated_obstacle = self.canvas.create_rectangle(x1 - self.inflation_radius,
                                                              y1 - self.inflation_radius,
                                                              x2 + self.inflation_radius,
                                                              y2 + self.inflation_radius,
                                                              fill="red", stipple="gray50")  # 인플레이션된 장애물 표시
            self.obstacles.append(inflated_obstacle)

    def update_collision_range(self):
        # 로봇의 충돌 범위 (회색 원) 위치 업데이트
        self.canvas.coords(self.robot_collision_range,
                        self.robot_x - self.robot_radius, self.robot_y - self.robot_radius,
                        self.robot_x + self.robot_radius, self.robot_y + self.robot_radius)

    def set_robot_position(self):
        # 로봇 위치를 입력받아서 설정
        try:
            new_x = int(self.robot_x_entry.get())
            new_y = int(self.robot_y_entry.get())
            if self.is_valid((new_x, new_y)):
                self.robot_x = new_x
                self.robot_y = new_y
                # 로봇의 위치 업데이트
                self.canvas.coords(self.robot, self.robot_x - self.robot_size, self.robot_y - self.robot_size,
                                self.robot_x + self.robot_size, self.robot_y + self.robot_size)
                # 로봇 방향 화살표 업데이트
                self.canvas.coords(self.robot_direction, self.robot_x, self.robot_y, self.robot_x + 30, self.robot_y)
                # 충돌 범위 원 업데이트
                self.update_collision_range()  # 충돌 범위 업데이트 함수 호출
                self.log_text.insert(tk.END, f"Robot position set to ({self.robot_x}, {self.robot_y}).\n")
            else:
                self.log_text.insert(tk.END, "Invalid position. Please select a valid location.\n")
        except ValueError:
            self.log_text.insert(tk.END, "Invalid input for position.\n")
        
    def set_obstacles(self):
        # 입력받은 값으로 장애물 설정
        try:
            self.inflation_radius = int(self.inflation_radius_entry.get())
            self.num_obstacles = int(self.num_obstacles_entry.get())
            self.refresh_obstacles()
        except ValueError:
            self.log_text.insert(tk.END, "Invalid input for obstacles.\n")
        
    def refresh_goal(self):
        # 기존 목표 지점 제거하고 새로운 목표 지점을 생성
        self.canvas.delete(self.goal)
        self.goal = self.place_goal()

    def refresh_obstacles(self):
        # 기존 장애물 제거하고 새로운 장애물 생성
        for obstacle in self.obstacles:
            self.canvas.delete(obstacle)
        self.place_obstacles()
    
    def randomize_robot_position(self):
        # 로봇 위치 랜덤화, 장애물이나 벽과 겹치지 않는지 확인
        valid_position = False
        while not valid_position:
            self.robot_x = random.randint(50, 350)
            self.robot_y = random.randint(50, 350)
            valid_position = self.is_valid((self.robot_x, self.robot_y))
        
        # 로봇의 위치 업데이트
        self.canvas.coords(self.robot, self.robot_x - self.robot_size, self.robot_y - self.robot_size,
                           self.robot_x + self.robot_size, self.robot_y + self.robot_size)
        self.canvas.coords(self.robot_direction, self.robot_x, self.robot_y, self.robot_x + 30, self.robot_y)
            # 로봇 방향 화살표 업데이트
        self.canvas.coords(self.robot_direction, self.robot_x, self.robot_y, self.robot_x + 30, self.robot_y)
        
        # 충돌 범위 원 업데이트
        self.update_collision_range()  # 충돌 범위 업데이트 함수 호출
        self.log_text.insert(tk.END, f"Robot position randomized to ({self.robot_x}, {self.robot_y}).\n")

    def is_valid(self, position):
        # 유효한 위치인지 확인 (장애물 또는 벽에 충돌하지 않는지)
        if position in self.walls:
            return False
        
        # 로봇의 충돌 범위 (로봇이 지나갈 수 있는 최소 거리)
        for obstacle in self.obstacles:
            coords = self.canvas.coords(obstacle)
            # 장애물 주변에 여유 공간을 두기 위해 로봇의 반경보다 큰 범위를 확인
            if (coords[0] - self.robot_radius <= position[0] <= coords[2] + self.robot_radius and
                coords[1] - self.robot_radius <= position[1] <= coords[3] + self.robot_radius):
                return False
        
        return True

    def run_pathfinding(self):
        # 경로 찾기 알고리즘 (A* 알고리즘)
        start = (self.robot_x, self.robot_y)
        goal = (self.canvas.coords(self.goal)[0] + 5, self.canvas.coords(self.goal)[1] + 5)
        path = self.a_star(start, goal)
        
        # 경로를 따라 로봇 이동
        if path:
            # 스레드로 경로를 따라 로봇을 이동
            threading.Thread(target=self.follow_path, args=(path,)).start()
        
    def a_star(self, start, goal):
        # A* 경로 찾기 알고리즘
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        open_list = []
        heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_list:
            _, cost, current = heapq.heappop(open_list)

            # 목표 위치 확인 (디버깅용)
            if abs(current[0] - goal[0]) < self.grid_size and abs(current[1] - goal[1]) < self.grid_size:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
            
            for dx, dy in [(-self.grid_size, 0), (self.grid_size, 0), (0, -self.grid_size), (0, self.grid_size)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if self.is_valid(neighbor):
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score[neighbor], tentative_g_score, neighbor))

        return []  # 경로가 없을 경우 빈 리스트 반환

    def follow_path(self, path):
        # 경로를 따라 로봇을 이동하는 함수
        for (x, y) in path:
            self.move_robot(x, y)
            time.sleep(0.05)


    def move_robot(self, x, y):
        # 로봇 이동
        dx = x - self.robot_x
        dy = y - self.robot_y
        
        # 로봇의 새 위치 계산
        self.robot_x = x
        self.robot_y = y
        
        # 로봇을 새로운 위치로 이동
        self.canvas.move(self.robot, dx, dy)
        
        # 로봇의 방향을 화살표로 표시
        radian = math.atan2(dy, dx)
        self.canvas.coords(self.robot_direction, self.robot_x, self.robot_y,
                        self.robot_x + 30 * math.cos(radian), self.robot_y + 30 * math.sin(radian))
        
        self.log_text.insert(tk.END, f"Robot moved to ({self.robot_x:.2f}, {self.robot_y:.2f}).\n")
        
        # 목표 지점 도달 확인
        if abs(self.robot_x - self.canvas.coords(self.goal)[0]) < 10 and abs(self.robot_y - self.canvas.coords(self.goal)[1]) < 10:
            self.log_text.insert(tk.END, "Goal reached!\n")
            # 충돌 범위 원 업데이트
            self.update_collision_range()  # 충돌 범위 업데이트 함수 호출
            self.log_text.insert(tk.END, f"Robot moved to goal ({self.robot_x}, {self.robot_y}).\n")
            return True
        
        
        # 충돌 범위 원 업데이트
        self.update_collision_range()  # 충돌 범위 업데이트 함수 호출
        self.log_text.insert(tk.END, f"Robot moved to goal ({self.robot_x}, {self.robot_y}).\n")

        return False

    def move_forward(self):
        # 현재 방향으로 로봇을 이동시킴
        radian = math.radians(self.robot_angle)
        new_x = self.robot_x + self.grid_size * math.cos(radian)
        new_y = self.robot_y + self.grid_size * math.sin(radian)
        
        if self.is_valid((new_x, new_y)):
            self.move_robot(new_x, new_y)
        else:
            self.log_text.insert(tk.END, "Move forward blocked by wall or obstacle.\n")
    
    def move_backward(self):
        # 현재 방향의 반대 방향으로 로봇을 이동시킴
        radian = math.radians(self.robot_angle)
        new_x = self.robot_x - self.grid_size * math.cos(radian)
        new_y = self.robot_y - self.grid_size * math.sin(radian)
        
        if self.is_valid((new_x, new_y)):
            self.move_robot(new_x, new_y)
        else:
            self.log_text.insert(tk.END, "Move backward blocked by wall or obstacle.\n")

    def rotate_robot(self):
        # 로봇을 90도 회전시킴
        self.robot_angle = (self.robot_angle + 90) % 360
        
        # 회전 후 로봇 방향 업데이트
        radian = math.radians(self.robot_angle)
        self.canvas.coords(self.robot_direction, self.robot_x, self.robot_y,
                           self.robot_x + 30 * math.cos(radian), self.robot_y + 30 * math.sin(radian))
        self.log_text.insert(tk.END, f"Robot rotated to {self.robot_angle} degrees.\n")


if __name__ == "__main__":
    app = VirtualRoboticsLab()
    app.mainloop()
