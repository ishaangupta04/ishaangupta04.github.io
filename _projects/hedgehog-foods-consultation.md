---
title: "YC Startup Consultation Project"
order: 2
excerpt: "Consulted a YC22 Agritech Startup"
header:
  image: /assets/images/A-Lab/frontal.pnh
  teaser: assets/images/A-lab/Cap_Dispenser_Main.jpg
sidebar:
  - title: "Role"
    image: 
    image_alt: 
    text: "Project Manager & Senior Engineer"
  - title: "Responsibilities"
    text: "Re-Design Armor Panels to enable configurations and shock reduction"
gallery:
  - url:
    image_path: 
    alt: "placeholder image 1"
  - url: 
    image_path: 
    alt: "placeholder image 2"
  - url: 
    image_path: 
    alt: "placeholder image 3"
---

## Introduction
  The purpose of this project was to improve upon the current cap dispenser system by reducing its failure rate and increasing the total holding capacity of the dispenser. The issues with the current dispenser included jams caused by an active cap feeder mechanism and a tendency for caps to nest when left in a tower for long periods of time. The re-design of this system targeted these issues in two ways, first by making the entire system passive, allowing gravity to feed the caps into a position where the robot arm could reliably pick up from, and second, by rotating the caps 90 degrees, such that sides of the cap be the contact edge with other caps. This entirely eliminates the issues with nesting, however it comes at the cost of the caps taking more space vertically.  The overall goal of this project was to implement these changes to improve the reliability of the system by reducing mechanical complexity and moving parts.

## Design Considerations
- Cap Capacity
  - 60-100 Caps
- Cap Alignment 
  - Allow for repeatable robot arm pick ups
- Cap Detection
  - Track Cap Capacity with beam break or infrared sensors
- Passive Cap Feeding
  - Reduces Failure Points
  - Simplifies Programming Complexity at the cost of Mechanical Complexity
- 3-D Printable Body
  - Allows for rapid iteration of Designs
  - Reduces part count
  - Reduces Assembly Time
- Modular Design
  - Reduces the size of parts to fit on 3-D Printer Build Plate
  - Allows for faster repairs 
  - Allows for expansion of system to accommodate larger or smaller volumes

## Prototype & Testing
This is the CAD used to eliminate Failure Mode 2. The additional slot reduces friction on the leading edge of the cap, which is what makes the most contact with the ramp floor. If the 3-D printer was able to print finer layers, then this fix would be entirely unnecessary. However, testing showed that gaps within the layers of the printing process caused imperfections which would catch the front edge of the caps as they contact the ramp, causing them to be stuck in place as pictured in Failure Mode 2. 

Failure Mode 1 was treated partially by the fix to Failure Mode 2. By increasing the angle of the ramp, this allowed for the caps to gain more speed on their way down the ramp, eliminating this failure.


## Final Design
  Pictured above is the finalized design of the cap dispenser. The flow of caps is as follows: The system will initially be completely full with caps. The system is designed to hold 70 caps, with 22 held in the magazine, and another 4 in the Main Cap Channel and the Dispensing Ramp.  As the robot takes a cap from the Pick Up Platform, the cap behind it will fall forward taking its place, and a cap will fall down from the magazine of the first stage. This will repeat until the first stage magazine is completely depleted. This status will be determined by the black beam break sensors, placed in the transition between the Main Cap Channel and the Dispensing Ramp. Then the linear actuator between the first and second stage will open, releasing the caps from the second stage into the Pick Up Platform. This process will continue until there are no more caps left in the system. The 2nd and 3rd Stages are identical to the first stage with the Dispensing Ramp removed, and are modular, allowing for more stages to be added or removed in the future. Large voids were placed in the structures to optimize 3-D printing speeds and material consumption. The system thus far has a 100% success rate in feeding the caps to the human hand with a sample size of 60 caps being fed through the system. To insure a repeatable pick up point for the robot arm, the Pick Up Platform has 2 angled walls that are designed to tangent to Cap Dispenser with a clearance of .01 inches. The cap behind the cap in Pick Up Platform applies a force on the first cap such that it is perfectly positioned.

## Remaining Steps
While the mechanical design of the project has been finalized, the programming of the sensors and actuators, as well as completing the associated electrical systems is required. Once this is completed, a live test with the robot arm needs to be conducted to test if the new system is better than the previous one.
