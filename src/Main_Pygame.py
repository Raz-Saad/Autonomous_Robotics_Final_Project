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
        self.map_width = 1366 #1000
        self.map_height = 768 #600
        self.screen_width = self.map_width + 100  # Increase width by some pixels
        self.screen_height = self.map_height + 32  # Increase height by some pixels
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.last_key_state = pygame.key.get_pressed() #for button pressing
        pygame.display.set_caption("Drone Simulation")
        self.show_legend = True  # Flag to control legend visibility

        # 3d expantion:
        self.ceiling_level = int(500 / 2.5) # 500 in cm, so dividing bt 2.5 for pixels
        self.floor_level = 0

        # Load map
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parent_directory = os.path.dirname(script_dir)
        input_filepath = os.path.join(parent_directory, 'maps')
        self.load_map_paths(input_filepath)  # Update with the correct path to your maps folder
        self.load_map(self.map_paths[self.current_map_index])

        self.sensor_texts = {
            "upward": "Upward: 0 cm",
            "downward": "Downward: 0 cm",
            "forward": "Forward: 0 cm",
            "backward": "Backward: 0 cm",
            "leftward": "Left: 0 cm",
            "rightward": "Right: 0 cm",
            "forward_right_diagonal":"forward_right_diagonal: 0 cm" ,
            "forward_left_diagonal": "forward_left_diagonal: 0 cm",
            "Distance_Sensors": "Distance Sensors:",
            "Height": "Drone's height: 0 cm",
            "Autonomous_Mode": "Autonomous Mode: True",
            "IMU": "IMU: 0",
            "Drone's battery": "0 %",
            "Drone's speed": "0",
            "Yellow_Percentage": "Yellow Percentage: 0.00%"
        }

        # Initialize drone
        self.drone_radius = int(10 / 2.5)  # Convert cm to pixels
        self.initial_drone_radius = self.drone_radius  # for saving the initial radius
        self.drone = Drone()
        self.drone_pos = None

        self.obstacles = [] # Initialize obstacles for the drone to fly above

        self.respawn_drone()

        self.amount_of_obstacles_to_spawn = 5
        self.spawn_obstacles(self.amount_of_obstacles_to_spawn)

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
            drone_z_level = (self.ceiling_level + self.floor_level) / 2
            self.drone.z_level = drone_z_level
            if not self.check_collision(x, y):
                self.drone_pos = [x, y]
                self.adjust_drone_radius()
                break


    def update_drone_desired_wall_distance(self):
        deviation = self.calculate_drone_z_axis_deviation()
        self.drone.update_desired_wall_distance(deviation)

    def calculate_drone_z_axis_deviation(self):
        z_epsilon_value = 17
        z_middle_value = (self.ceiling_level + self.floor_level) / 2
        return (self.drone.z_level - (z_middle_value-z_epsilon_value)) / z_middle_value #(self.ceiling_level / 2)

    # adjust the drone's radius by it's z axis, for visualization:
    def adjust_drone_radius(self):
        # the drone's radius will be it's initial value + the deviation of its actual height from the middle height value:
        self.drone_radius = int(self.initial_drone_radius + # initial value
                             (self.calculate_drone_z_axis_deviation() * self.initial_drone_radius)) # deviation precentage

    def check_map_z_collision(self):
        if self.drone.z_level <= self.floor_level or self.drone.z_level >= self.ceiling_level: # out of height bound
            return True
        return False

    # checking collision with floor/ceiling/walls/obstacles:
    def check_collision(self, x, y, radius=None):
        # check floor/ceiling collision:
        if self.check_map_z_collision():
            return True

        if radius is None:
            radius = self.drone_radius

        # check walls collision:
        for i in range(int(x - radius), int(x + radius)):
            for j in range(int(y - radius), int(y + radius)):
                if 0 <= i < self.map_width and 0 <= j < self.map_height:
                    if self.map_matrix[j][i] == 1:
                        return True

                    #check obstacles collision:
                    for obstacle in self.obstacles:
                        ox, oy, oradius, _, oheight = obstacle
                        if math.sqrt((i - ox)**2 + (j - oy)**2) < oradius:
                            # Check z-level for collision
                            if self.drone.z_level <= oheight:
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


    # for controling the drone's height:
    def move_drone_z_axis(self,z_direction):
        self.drone.change_z_level(z_direction)
        self.check_move_legality(self.drone_pos)


    # move with user input keys
    def move_drone_by_direction(self, direction = "forward"):  
        new_pos = self.drone.move_drone(self.drone_pos , direction)
        self.check_move_legality(new_pos)

    # for controling the drone manually:
    def update_drone_angle(self, angle_delta):
        self.drone.update_drone_angle(angle_delta)


    def is_obstacle_colliding_with_drone_xy(self,obstacle):
        ox,oy,obstacle_radius,o_color,o_height = obstacle

        d_x , d_y = self.drone_pos

        # Calculate distance between drone center (self.x, self.y) and obstacle center (ox, oy)
        distance = math.sqrt((d_x - ox) ** 2 + (d_y - oy) ** 2)

        # Check collision: if distance <= sum of radii (self.drone_radius + obstacle_radius)
        if distance <= (self.drone_radius + obstacle_radius):
            return True
        else:
            return False



    def update_sensors(self):
        # gathering x,y colliding obstacles, for the distance-hight sensors:
        colliding_obstacles = []
        for obstacle in self.obstacles:
            if self.is_obstacle_colliding_with_drone_xy(obstacle):
                colliding_obstacles.append(obstacle)

        # updating drone's sensors:
        self.drone.update_sensors(self.map_matrix, self.drone_pos, self.drone_radius,
                                  self.drone.orientation_sensor.drone_orientation,
                                  self.floor_level,self.ceiling_level,
                                  colliding_obstacles) # for updating up/down distance sensors
       
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
        #reseting flags for the autonomous flight algorithm:
        self.drone.returning_to_start = False
        self.drone.returning_to_explore = False
        self.drone.charging_drone = False

        self.drone.return_to_exploring_path.clear() # clearing previously saved return to explore paths
        self.drone.return_home_path.clear() # clearing previously saved return home paths

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

        self.obstacles.clear()  # Clear obstacles
        self.spawn_obstacles(self.amount_of_obstacles_to_spawn)  # Respawn obstacles
        
    def calculate_yellow_percentage(self):
        yellow_pixels_count = len(self.detected_yellow_pixels)
        percentage = (yellow_pixels_count / self.total_white_pixels) * 100
        return 100.0 if percentage > 100 else percentage

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
            "1/2: Increase/Decrease Height",
            "H: Return Home",
            "M: Change Map"
        ]

        # Create a semi-transparent background for the legend
        legend_surface = pygame.Surface((400, 400))
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

    def calculate_height_from_grey_shade(self,grey_shade):
        height = int(self.floor_level + # base height
                    (self.ceiling_level - self.floor_level) * ((255.0 - grey_shade) / 255.0)) # added value that wont go more than the ceiling
        return height

    def spawn_obstacles(self, num_obstacles):
        for _ in range(num_obstacles):
            while True:
                obstacle_radius = random.randint(20, 30)
                x = random.randint(obstacle_radius, self.map_width - obstacle_radius)
                y = random.randint(obstacle_radius, self.map_height - obstacle_radius)
                if not self.check_collision(x, y, obstacle_radius):
                    grey_shade = 200 # 0 = black = max height, 255 = white = min height
                    color = (grey_shade, grey_shade, grey_shade)
                    # Calculate obstacle height based on grey shade
                    height = self.calculate_height_from_grey_shade(grey_shade)

                    # making "stairs" obstacles:
                    amount_of_stairs = 4 # layers of heights per obstacle
                    for i in range(amount_of_stairs):
                        layer_grey_shade = min( 240, grey_shade - i*30)
                        layer_grey_shade = max( 20, layer_grey_shade)
                        layer_height = self.calculate_height_from_grey_shade(layer_grey_shade)
                        self.obstacles.append((x, y, int(obstacle_radius *((amount_of_stairs-i)/amount_of_stairs)) , (layer_grey_shade,layer_grey_shade,layer_grey_shade), layer_height))
                        
                    break

    def draw_obstacles(self):
        for obstacle in self.obstacles:
            x, y, radius, color, _ = obstacle
            pygame.draw.circle(self.screen, color, (x, y), radius)

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

            # 3d expantion:
            if keys[pygame.K_1]:
                self.move_drone_z_axis(1)

            if keys[pygame.K_2]:
                self.move_drone_z_axis(-1)

            if keys[pygame.K_h]:
                print("h was pressed, returning home...")
                self.drone.returning_to_start = True

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

            # Check if it's time to update the sensors , we want to update 10 times per second
            current_time = pygame.time.get_ticks()
            if current_time - sensors_update_timer >= interval:
                # Update sensors
                self.update_sensors()
                sensors_update_timer = current_time  # Reset the timer

            #adjust the drone's radius by it's current height:
            self.adjust_drone_radius()

            if is_autonomous:
                self.update_drone_desired_wall_distance() # adjusted based on the drone's changing size

                # Update drone position by algorithm
                self.drone_pos = self.drone.update_position_by_algorithm(
                    self.drone_pos,
                    dt,
                    self.map_matrix,
                    self.drone_radius)
            else:
                self.move_drone_by_direction()
            #checking if the drone crashing into the wall or not  
            self.check_move_legality(self.drone_pos)

            # Blit the map image onto the screen
            self.screen.blit(self.map_img, (0, 0))

            # Paint detected points (yellow markers)
            self.paint_detected_points()

            yellow_percentage=self.calculate_yellow_percentage()

            # Draw obstacles
            self.draw_obstacles()

            # this calculation get the factor of change the circle itself had, and we will use it to change the drone's arrow as well:
            precentage_of_drone_height_deviation = (self.drone_radius / self.initial_drone_radius)

            # Blit the drone trail onto the screen
            for pos in self.drone_positions:
                pygame.draw.circle(self.screen, (0, 0, 255), pos, 2)

            # Draw arrow on the drone indicating its direction
            angle_rad = math.radians(self.drone.orientation_sensor.drone_orientation)
            end_x = self.drone_pos[0] + 15 * precentage_of_drone_height_deviation * math.cos(angle_rad)
            end_y = self.drone_pos[1] + 15 * precentage_of_drone_height_deviation * math.sin(angle_rad)
            pygame.draw.line(self.screen, (0, 0, 0), self.drone_pos, (end_x, end_y), int(2* precentage_of_drone_height_deviation) )

            # Blit the drone onto the screen
            pygame.draw.circle(self.screen, (255, 0, 0), self.drone_pos, self.drone_radius)


            # Update sensor texts
            self.sensor_texts["upward"] = f"Upward: {self.drone.up_distance_sensor.distance *2.5:.1f} cm"
            self.sensor_texts["downward"] = f"Downward: {self.drone.down_distance_sensor.distance*2.5:.1f} cm"
            self.sensor_texts["forward"] = f"Forward: {self.drone.forward_distance_sensor.distance:.1f} cm"
            self.sensor_texts["backward"] = f"Backward: {self.drone.backward_distance_sensor.distance:.1f} cm"
            self.sensor_texts["leftward"] = f"Left: {self.drone.leftward_distance_sensor.distance:.1f} cm"
            self.sensor_texts["rightward"] = f"Right: {self.drone.rightward_distance_sensor.distance:.1f} cm"
            self.sensor_texts["IMU"] = f"IMU: {(360 - self.drone.orientation_sensor.drone_orientation) % 360:.1f}"
            self.sensor_texts["Drone's battery"] = f"Drone's battery: {self.drone.battery_sensor.get_battrey_precentage():.1f} %"
            self.sensor_texts["Drone's speed"] = f"Drone's speed: {self.drone.optical_flow_sensor.get_current_speed():.1f}"
            self.sensor_texts["Yellow_Percentage"] = f"Yellow_Percentage: {yellow_percentage:.2f} %"
            self.sensor_texts["Autonomous_Mode"] = f"Autonomous_Mode: {is_autonomous}"
            self.sensor_texts["Height"] = f"Drone's height: {self.drone.z_level *2.5:.1f} cm"
            self.sensor_texts["forward_right_diagonal"] = f"forward_right_diagonal: {self.drone.forward_right_diagonal_distance_sensor.distance:.1f} cm"
            self.sensor_texts["forward_left_diagonal"] = f"forward_left_diagonal: {self.drone.forward_left_diagonal_distance_sensor.distance:.1f} cm"

            # Display sensor texts
            font_size = 24
            font = pygame.font.SysFont(None, font_size)
            for i, (key, text) in enumerate(self.sensor_texts.items()):
                if key == "Distance_Sensors":
                    font.set_underline(True)
                    font.set_bold(True)
                else:
                    font.set_underline(False)
                    font.set_bold(False)
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
