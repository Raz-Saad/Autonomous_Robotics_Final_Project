import pygame
from PIL import Image
import random
import math
import os
from Drone import Drone
import time

# DroneSimulation class
class DroneSimulation:
    def __init__(self):
        pygame.init()
        self.map_width = 1366
        self.map_height = 768
        self.screen_width = self.map_width + 100  # Increase width by some pixels
        self.screen_height = self.map_height + 32  # Increase height by some pixels
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.last_key_state = pygame.key.get_pressed() #for button pressing
        pygame.display.set_caption("Drone Simulation")
        self.show_legend = True  # Flag to control legend visibility

        # Load map
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parent_directory = os.path.dirname(script_dir)
        input_filepath = os.path.join(parent_directory, 'maps')
        self.load_map_paths(input_filepath)  # Update with the correct path to your maps folder
        self.load_map(self.map_paths[self.current_map_index])

        self.sensor_texts = {
            "Autonomous_Mode": "Autonomous Mode: True",
            "forward": "Forward: 0 cm",
            "backward": "Backward: 0 cm",
            "leftward": "Left: 0 cm",
            "rightward": "Right: 0 cm",
            "Distance_Sensors": "Distance Sensors:",
            "IMU": "IMU: 0",
            "Drone's battery": "0 %",
            "Drone's speed": "0",
            "Yellow_Percentage": "Yellow Percentage: 0.00%"
        }

        # Initialize drone
        self.drone_radius = int(10 / 2.5)  # Convert cm to pixels
        self.drone = Drone()
        self.drone_pos = None
        self.respawn_drone()

        self.clock = pygame.time.Clock()
        self.game_over = False

        # List to store drone positions for leaving a trail
        self.drone_positions = []

        # Array to remember painted pixels
        self.detected_pixels = set()
        self.detected_yellow_pixels = set()

        # Count initial white pixels
        self.total_white_pixels = self.count_white_pixels()

        self.drone.set_starting_position(self.drone_pos)

    def load_map(self, filename):
        map_img = Image.open(filename)
        map_img = map_img.resize((self.map_width, self.map_height))
        self.map_img = pygame.image.fromstring(map_img.tobytes(), map_img.size, map_img.mode)
        self.map_matrix = self.convert_to_matrix(map_img)

    def convert_to_matrix(self, img):
        bw_img = img.convert("L")  # Convert to grayscale
        threshold = 128  # Threshold value for black/white
        bw_matrix = []
        for y in range(self.map_height):
            row = []
            for x in range(self.map_width):
                pixel = bw_img.getpixel((x, y))
                if pixel < threshold:
                    row.append(1)  # Black pixel
                else:
                    row.append(0)  # White pixel
            bw_matrix.append(row)
        return bw_matrix
    
    def count_white_pixels(self):
        count = 0
        for row in self.map_matrix:
            count += row.count(0)
        return count

    def respawn_drone(self):
        while True:
            x = random.randint(self.drone_radius, self.map_width - self.drone_radius - 1)
            y = random.randint(self.drone_radius, self.map_height - self.drone_radius - 1)
            if not self.check_collision(x, y):
                self.drone_pos = [x, y]
                break

    def check_collision(self, x, y):
        for i in range(int(x - self.drone_radius), int(x + self.drone_radius)):
            for j in range(int(y - self.drone_radius), int(y + self.drone_radius)):
                if 0 <= i < self.map_width and 0 <= j < self.map_height:
                    if self.map_matrix[j][i] == 1:
                        return True
        return False

    # checks if the drone is inside the map and also is not collided, if it did reset the game
    def check_move_legality(self , new_pos):
        if (self.drone_radius <= new_pos[0] < self.map_width - self.drone_radius and
                self.drone_radius <= new_pos[1] < self.map_height - self.drone_radius):
            if not self.check_collision(new_pos[0], new_pos[1]):
                self.drone_pos = new_pos
                self.drone.update_position(new_pos)  # Track the trail
                self.drone_positions.append(self.drone_pos[:])  # Add position to the trail
            else:
                self.reset_simulation()
        else:
            self.reset_simulation()

    # move with user input keys
    def move_drone_by_direction(self, direction = "forward"):  
        new_pos = self.drone.move_drone(self.drone_pos , direction)
        self.check_move_legality(new_pos)
    # for controling the drone manul
    def update_drone_angle(self, angle_delta):
        self.drone.update_drone_angle(angle_delta)

    def update_sensors(self):
        self.drone.update_sensors(self.map_matrix, self.drone_pos, self.drone_radius, self.drone.orientation_sensor.drone_orientation)
       
    def paint_detected_points(self):
        def get_detected_points(sensor_distance, angle_offset):
            angle_rad = math.radians((self.drone.orientation_sensor.drone_orientation + angle_offset) % 360)
            points = []
            for dist in range(1, int(min(sensor_distance, 300) / 2.5) + 1):
                x = self.drone_pos[0] + dist * math.cos(angle_rad)
                y = self.drone_pos[1] + dist * math.sin(angle_rad)
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    if self.map_matrix[int(y)][int(x)] == 0:  # Check if the point is in the white area
                        points.append((int(x), int(y)))
            return points

        # Get detected points for left and right sensors
        left_points = get_detected_points(self.drone.leftward_distance_sensor.distance, -90)
        right_points = get_detected_points(self.drone.rightward_distance_sensor.distance, 90)

        # Calculate the new points to be added
        new_detected_points = set(left_points + right_points)
        points_to_paint = new_detected_points - self.detected_pixels

        # Function to get all points within a radius of 2 around a point
        def get_points_in_radius(x, y, radius=2):
            points = set()
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if dx * dx + dy * dy <= radius * radius:
                        new_x, new_y = x + dx, y + dy
                        if 0 <= new_x < self.map_width and 0 <= new_y < self.map_height:
                            if self.map_matrix[new_y][new_x] == 0:  # Check if the point is in the white area
                                points.add((new_x, new_y))
            return points
    

        # Update the main set of detected pixels with the new points and their surrounding points
        expanded_points_to_paint = set()
        for x, y in points_to_paint:
            expanded_points_to_paint.update(get_points_in_radius(x, y))

        self.detected_pixels.update(expanded_points_to_paint)

        # Create a surface for detected points if not exists
        if not hasattr(self, 'detected_surface'):
            self.detected_surface = pygame.Surface((self.map_width, self.map_height))
            self.detected_surface.set_colorkey((0, 0, 0))  # Set transparent color

        # Paint only the new points on the detected surface
        for x, y in expanded_points_to_paint:
           self.detected_surface.set_at((x, y), (255, 255, 0))

        # Store detected points for yellow collection
        self.detected_yellow_pixels.update(expanded_points_to_paint)

        # Blit the detected surface onto the main screen
        self.screen.blit(self.detected_surface, (0, 0))

    def reset_simulation(self):
        self.detected_pixels.clear()  # Clear detected points
        self.respawn_drone()  # Respawn the drone
        self.drone_positions.clear()  # Clear trail
        # empty the surface, ensuring that no previously detected points are displayed on the screen.
        if hasattr(self, 'detected_surface'):
            self.detected_surface.fill((0, 0, 0))  # Fill the detected surface with black color
        self.drone.optical_flow_sensor.reset_sensor()
        self.drone.battery_sensor.reset_battrey()
        self.detected_yellow_pixels.clear()  # Clear yellow detected points
        self.sensor_texts["Yellow_Percentage"] = "Yellow Percentage: 0.00%"
        #making the drone start flying
        self.drone.optical_flow_sensor.update_speed_acceleration()
        #clearing the drone trail array for wall switching
        self.drone.trail.clear()
        
    def calculate_yellow_percentage(self):
        yellow_pixels_count = len(self.detected_yellow_pixels)
        percentage = (yellow_pixels_count / self.total_white_pixels) * 100
        return percentage

    def draw_legend_menu(self):
        # Define the legend text
        legend_texts = [
            "Controls Legend:",
            "L: Show/Hide Legend",
            "Arrow Keys: Move Drone",
            "A/D: Rotate Drone",
            "Q: Switch Wall",
            "R: Reset Simulation",
            "E: Toggle Autonomous Mode",
            "W: Increase Speed",
            "S: Decrease Speed",
            "M: Change Map"
        ]

        # Create a semi-transparent background for the legend
        legend_surface = pygame.Surface((400, 300))
        legend_surface.set_alpha(200)  # Transparency
        legend_surface.fill((50, 50, 50))

        # Render the legend text
        font = pygame.font.SysFont(None, 24, bold=True)
        for i, text in enumerate(legend_texts):
            text_surface = font.render(text, True, (255, 255, 255))
            legend_surface.blit(text_surface, (10, 10 + i * 30))

        # Blit the legend surface onto the screen
        self.screen.blit(legend_surface, (50, 50))

    def load_map_paths(self, folder_path):
        self.map_paths = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith(('png', 'jpg', 'jpeg'))]
        self.current_map_index = 0

    def load_next_map(self):
        self.current_map_index = (self.current_map_index + 1) % len(self.map_paths)
        self.load_map(self.map_paths[self.current_map_index])
        self.reset_simulation()


    def run_simulation(self):
        
        # Define the desired frequency (10 times per second)
        frequency = 10  # Hz
        interval = 1000 // frequency  # Convert frequency to milliseconds
        sensors_update_timer = pygame.time.get_ticks()  # Initialize timer for sensors update
        last_time = time.time()
        is_autonomous = True # a flag for enabling/disabling autonomous flight mode
        #PID_value_change = 0.005
        #making the drone start flying
        self.drone.optical_flow_sensor.update_speed_acceleration()
        while not self.game_over:

            current_time_test = time.time()
            dt = current_time_test - last_time
            last_time = current_time_test

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.game_over = True

            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                self.move_drone_by_direction("backward")
            if keys[pygame.K_RIGHT]:
                self.move_drone_by_direction("forward")
            if keys[pygame.K_UP]:
                self.move_drone_by_direction("leftward")
            if keys[pygame.K_DOWN]:
                self.move_drone_by_direction("rightward")
            if keys[pygame.K_a]:
                self.update_drone_angle(-10)
            if keys[pygame.K_d]:
                self.update_drone_angle(10)
            # Toggle functionality keys
            if keys[pygame.K_q] and not self.last_key_state[pygame.K_q]:
                self.drone.switch_wall()
            if keys[pygame.K_r] and not self.last_key_state[pygame.K_r]:
                self.reset_simulation()
            if keys[pygame.K_e] and not self.last_key_state[pygame.K_e]:
                is_autonomous = not is_autonomous
            if keys[pygame.K_l] and not self.last_key_state[pygame.K_l]:
                self.show_legend = not self.show_legend  # Toggle the legend visibility
            if keys[pygame.K_m] and not self.last_key_state[pygame.K_m]:  # Key to change the map
                self.load_next_map()
            # Check if the right arrow key is pressed to update the speed
            if keys[pygame.K_w] and not self.last_key_state[pygame.K_w]:
                self.drone.optical_flow_sensor.update_speed_acceleration()
            if keys[pygame.K_s] and not self.last_key_state[pygame.K_s]:
                self.drone.optical_flow_sensor.update_speed_deceleration()

            # Update the last key state
            self.last_key_state = keys

            # #FOR UPDATING P
            # if keys[pygame.K_1]:
            #     self.drone.pid_controller.update_P_value(PID_value_change)
            # if keys[pygame.K_2]:
            #     self.drone.pid_controller.update_P_value(-PID_value_change)
            # #FOR UPDATING I    
            # if keys[pygame.K_3]:
            #     self.drone.pid_controller.update_I_value(PID_value_change)
            # if keys[pygame.K_4]:
            #     self.drone.pid_controller.update_I_value(-PID_value_change)
            # #FOR UPDATING D
            # if keys[pygame.K_5]:
            #     self.drone.pid_controller.update_D_value(PID_value_change)
            # if keys[pygame.K_6]:
            #     self.drone.pid_controller.update_D_value(-PID_value_change)

            # Check if it's time to update the sensors , we want to update 10 times per second
            current_time = pygame.time.get_ticks()
            if current_time - sensors_update_timer >= interval:
                # Update sensors
                self.update_sensors()
                sensors_update_timer = current_time  # Reset the timer
            
            if is_autonomous:
                # Update drone position by algorithm
                self.drone_pos = self.drone.update_position_by_algorithm(self.drone_pos, dt)
            else:
                self.move_drone_by_direction()
            #checking if the drone crashing into the wall or not  
            self.check_move_legality(self.drone_pos)
                
            # Blit the map image onto the screen
            self.screen.blit(self.map_img, (0, 0))

            # Paint detected points (yellow markers)
            self.paint_detected_points()

            yellow_percentage=self.calculate_yellow_percentage()

            # Blit the drone trail onto the screen
            for pos in self.drone_positions:
                pygame.draw.circle(self.screen, (0, 0, 255), pos, 2)

            # Draw arrow on the drone indicating its direction
            angle_rad = math.radians(self.drone.orientation_sensor.drone_orientation)
            end_x = self.drone_pos[0] + 15 * math.cos(angle_rad)
            end_y = self.drone_pos[1] + 15 * math.sin(angle_rad)
            pygame.draw.line(self.screen, (0, 0, 0), self.drone_pos, (end_x, end_y), 2)

            # Blit the drone onto the screen
            pygame.draw.circle(self.screen, (255, 0, 0), self.drone_pos, self.drone_radius)

            # Update sensor texts
            self.sensor_texts["forward"] = f"Forward: {self.drone.forward_distance_sensor.distance:.1f} cm"
            self.sensor_texts["backward"] = f"Backward: {self.drone.backward_distance_sensor.distance:.1f} cm"
            self.sensor_texts["leftward"] = f"Left: {self.drone.leftward_distance_sensor.distance:.1f} cm"
            self.sensor_texts["rightward"] = f"Right: {self.drone.rightward_distance_sensor.distance:.1f} cm"
            self.sensor_texts["IMU"] = f"IMU: {(360 - self.drone.orientation_sensor.drone_orientation) % 360:.1f}"
            self.sensor_texts["Drone's battery"] = f"Drone's battery: {self.drone.battery_sensor.get_battrey_precentage():.1f} %"
            self.sensor_texts["Drone's speed"] = f"Drone's speed: {self.drone.optical_flow_sensor.get_current_speed():.1f}"
            self.sensor_texts["Yellow_Percentage"] = f"Yellow_Percentage: {yellow_percentage:.2f} %"
            self.sensor_texts["Autonomous_Mode"] = f"Autonomous_Mode: {is_autonomous}"


            # Display sensor texts
            font_size = 24
            font = pygame.font.SysFont(None, font_size)
            for i, (key, text) in enumerate(self.sensor_texts.items()):
                text_surface = font.render(text, True, (0, 204, 0))
                self.screen.blit(text_surface, (0, self.map_height - 15 - i * 30))

            # Draw legend menu if the flag is set
            if self.show_legend:
                self.draw_legend_menu()

            pygame.display.update()
            self.clock.tick(60)

        pygame.quit()


def main():
    simulation = DroneSimulation()
    simulation.run_simulation()

if __name__ == "__main__":
    main()
