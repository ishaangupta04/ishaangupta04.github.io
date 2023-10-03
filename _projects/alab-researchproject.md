---
title: "A-Lab Research Project"
order: 1
excerpt: "Designing Robot Cells for Autonomous Material Synthesis Laboratory"
header:
  image: /assets/images/A-Lab/frontal.png
  teaser: /assets/images/A-lab/Cap_Dispenser_Main_render.jpg
sidebar:
  - title: "Role"
    image: 
    image_alt: 
    text: "Student Assistant"
  - title: "Responsibilities"
    text: "Designing Robot Cells for Autonomous Material Synthesis Laboratory"
toc: true
toc_label: "Table of Contents"
toc_sticky: true
    
# Prototyping
feature_row:
  - image_path: /assets/images/A-Lab/failure-mode-1.jpg
    excerpt: "Failure Mode 1"
  - image_path: /assets/images/A-Lab/failure-mode-2.jpg
    excerpt: "Failure Mode 2"
feature_row2:
  - image_path: /assets/images/A-Lab/failure-mode-1-sol-1.png
    excerpt: "Failure Mode 1 Solution"
  - image_path: /assets/images/A-Lab/failure-mode-1-sol-2.png
    excerpt: "Failure Mode 2 Solution"

#Final Design
feature_row3:
  - image_path: /assets/images/A-Lab/figure1.png
    title: "Figure 1"
    excerpt: "Section View of First Stage, showing the transition between the Cap Magazine and the Main Cap Channel. Shows placement of beam break sensors (Black) and Linear Actuators."
  - image_path: /assets/images/A-Lab/figure2.png
    title: "Figure 2"
    excerpt: "Zoomed in Assembly of the First Stage, showing the Pick Up Platform."
feature_row4:
  - image_path: /assets/images/A-Lab/figure3.png
    title: "Figure 3"
    excerpt: "Section View of First Stage, Showing the transition between the Main Cap Channel and the Dispensing Ramp. Shows the placement of beam break sensors (black)."
  - image_path: /assets/images/A-Lab/figure4.png
    title: "Figure 4"
    excerpt: "Full Assembly"
feature_row5:
  - image_path: /assets/images/A-Lab/figure5.png
    title: "Figure 5"
    excerpt: "Shown above is the parametric design of the Pick Up Platform."
---

## Introduction
  The purpose of this project was to improve upon the current cap dispenser system by reducing its failure rate and increasing the total holding capacity of the dispenser. The issues with the current dispenser included jams caused by an active cap feeder mechanism and a tendency for caps to nest when left in a tower for long periods of time. The re-design of this system targeted these issues in two ways, first by making the entire system passive, allowing gravity to feed the caps into a position where the robot arm could reliably pick up from, and second, by rotating the caps 90 degrees, such that sides of the cap be the contact edge with other caps. This entirely eliminates the issues with nesting, however it comes at the cost of the caps taking more space vertically.  The overall goal of this project was to implement these changes to improve the reliability of the system by reducing mechanical complexity and moving parts.

## Design Considerations
- Cap Capacity
  - 60-100 Caps
- Cap Alignment 
  - Allow for repeatable robot arm pick ups
- Cap Detection
  - Track cap caapacity with beam break or infrared sensors
- Passive Cap Feeding
  - Reduces Failure Points
  - Simplifies programming complexity at the cost of increased mechanical complexity
- 3-D Printable Body
  - Allows for rapid iteration
  - Reduces part count
  - Reduces assembly time
- Modular Design
  - Reduces the size of parts to fit on 3-D Printer build plate
  - Allows for faster repairs 
  - Allows for expansion of system to accommodate larger or smaller volumes

## Inital Concepts
{% include figure image_path="/assets/images/A-Lab/inital-concepts.jpg" alt="" caption="" %}
{% include figure image_path="/assets/images/A-Lab/concept1.png" alt="" caption="Proof of concept CAD for the cap magazine design and the use of linear actuators as gates" %}

## Prototype & Testing
{% include feature_row %}
{% include feature_row id= "feature_row2" %}
This is the CAD used to eliminate Failure Mode 2. The additional slot reduces friction on the leading edge of the cap, which is what makes the most contact with the ramp floor. If the 3-D printer was able to print finer layers, then this fix would be entirely unnecessary. However, testing showed that gaps within the layers of the printing process caused imperfections which would catch the front edge of the caps as they contact the ramp, causing them to be stuck in place as pictured in Failure Mode 2. 

Failure Mode 1 was treated partially by the fix to Failure Mode 2. By increasing the angle of the ramp, this allowed for the caps to gain more speed on their way down the ramp, eliminating this failure.


## Final Design
{% include feature_row id= "feature_row3" %}
{% include feature_row id= "feature_row4" %}
{% include feature_row id= "feature_row5" %}
  Pictured above is the finalized design of the cap dispenser. The flow of caps is as follows: The system will initially be completely full with caps. The system is designed to hold 70 caps, with 22 held in the magazine, and another 4 in the Main Cap Channel and the Dispensing Ramp.  As the robot takes a cap from the Pick Up Platform, the cap behind it will fall forward taking its place, and a cap will fall down from the magazine of the first stage. This will repeat until the first stage magazine is completely depleted. This status will be determined by the black beam break sensors, placed in the transition between the Main Cap Channel and the Dispensing Ramp. Then the linear actuator between the first and second stage will open, releasing the caps from the second stage into the Pick Up Platform. This process will continue until there are no more caps left in the system. The 2nd and 3rd Stages are identical to the first stage with the Dispensing Ramp removed, and are modular, allowing for more stages to be added or removed in the future. Large voids were placed in the structures to optimize 3-D printing speeds and material consumption. The system thus far has a 100% success rate in feeding the caps to the human hand with a sample size of 60 caps being fed through the system. To insure a repeatable pick up point for the robot arm, the Pick Up Platform has 2 angled walls that are designed to tangent to Cap Dispenser with a clearance of .01 inches. The cap behind the cap in Pick Up Platform applies a force on the first cap such that it is perfectly positioned.

## Remaining Steps
While the mechanical design of the project has been finalized, the programming of the sensors and actuators, as well as completing the associated electrical systems is required. Once this is completed, a live test with the robot arm needs to be conducted to test if the new system is better than the previous one.
