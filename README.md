# Tiny Drone Simulation Enhancement - Autonomous Robotics Final Project
## Submited by </br>
* Maor Or
* Raz Saad
* Shlomit Ashkenazi
  
## Overview
This project aims to expand the capabilities of the "tiny drone" simulation ([Assignment 1](https://github.com/Raz-Saad/Autonomous_Robotics_Ex1)). Our goal is to create a high-quality, open-source platform that supports the simulation of a control algorithm of a drone.</br>
Additionally, the drone will be capable of scanning an entire map efficiently, similar to a robotic mop cleaner.

## Features & Improvements
*  <b> Wall-Switching Mechanism </b>: an addition that helps the drone switch bettwen walls whenever it detects that it completed scanning that local area and switching walls make the drone scan new areas. 
*  <b> Improved Return to Start algorithm </b>: instead of returning to the starting point of the simulation by tracking back the drone's entrie trail. Now, given a point where the drone is located, it identifies all the points on its trail that are within the range of its sensors and also with line of sight, and chooses from them the point closest to the start. The drone then creates a path to that point and flies to it. The drone continues this process until it reaches the starting point.
*  <b> Drone Charging & Getting Back to Scanning </b>: When the drone battery is getting to 50% it gets back to is starting point. Instead of stopping the simulation, now the drone charges back to 100% battery and then returns to the last point it was exploring to continue the map scan from this point.
* <b> 2.5D simulation </b>:
  * Added a height paramter to the drone and ceilling & floor paramters to the map, in order to have a 3D space in 2D simulation.
  * Added height sensors that can detect distances of the drone from above and below obstacles.
  * Added height obstacles for the drone to fly above (drone's size visually changes accordingly).
  * Added a PID controller to control the drone's height.
*  <b> Control Algorithm Simulation </b> - Improved current control algorithm performance with the following:
  * Wall-Switching Mechanism (mentioned above).
  * More optimized PID controllers values, resulting in better drone flight and survivability.
  * Additional Sensors - added 2 front diagonal left and right sensors, making the drone have better response to its environment.  

## How to run
* Clone the repo to a local machine
* Download requirements with: ```pip install -r requirements.txt```
* From a terminal window navigate to the src folder and execute: ```python Main_Pygame.py```
* Optional - You can add a custom-made map for the drone to cover: add the image into the maps folder
  
## Demo
### Wall-Switching Mechanism


https://github.com/Raz-Saad/Autonomous_Robotics_Final_Project/assets/43138073/64b8a4d1-0d65-49bf-87a4-6cd879b04116

### Improved Return to Start algorithm + Drone Charging & Getting Back to Scanning (For the demo - battrey life time is 60 secs)



https://github.com/Raz-Saad/Autonomous_Robotics_Final_Project/assets/43138073/8023b8f7-ddd9-4f91-82c0-37cd281c19dd


### Flying Above Obstacles



https://github.com/Raz-Saad/Autonomous_Robotics_Final_Project/assets/43138073/e234f670-5cd2-40b1-ba92-7828ec7f5a35


## Previous Work
### Vision-Less Sensing for Autonomous Micro-Drones - by  Simon Pikalov,Elisha Azaria,Shaya Sonnenberg,Boaz Ben-Moshe  and Amos Azaria </br>
* [Link to the article](https://www.mdpi.com/1424-8220/21/16/5293) </br>

* This work is representing a concept of intelligent vision-less micro-drones, inspired by flying animals such as insects, birds, and bats. The presented micro-drone, named BAT (Blind Autonomous Tiny-drone), can perform bio-inspired complex tasks without the use of cameras. The BAT uses LIDARs and self-emitted optical flow to avoid obstacles and solve mazes. The controlling algorithms were implemented on an onboard microcontroller, allowing the BAT to be fully autonomous. Additionally, a method was developed to use the information collected by the drone to generate a detailed map of the environment. A complete model of the BAT was implemented and tested in various scenarios, both in simulation and field experiments, demonstrating its ability to explore and map complex buildings autonomously, even in total darkness.
