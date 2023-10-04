---
title: "YC22 Startup Consultation"
excerpt: "Scaling down a smart oven for agri-tech applications"
header:
  image: /assets/images/hedgehog/final.jpg
  teaser: assets/images/hedgehog/final.jpg
sidebar:
  - title: "Role"
    image: 
    image_alt: 
    text: "Project Manager and Engineering Lead"
  - title: "Responsibilities"
    text: "Project Mangement, BOM Mangement, Part Sourcing, CAD/CAM, PID Control"
toc: true
toc_label: "Table of Contents"
toc_sticky: true
    
# Prototyping
feature_row:
  - image_path: /assets/images/hedgehog/proto-inside-body.jpg
    excerpt: "Body Interior"
  - image_path: /assets/images/hedgehog/proto-body.jpg
    excerpt: "Body Exterior"
feature_row2:
  - image_path: /assets/images/hedgehog/proto-motor.jpg
    excerpt: "3-D Printed Motor Bracket"

#Final Design
feature_row3:
  - image_path: /assets/images/hedgehog/final-top.jpg
    excerpt: "Section View of First Stage, showing the transition between the Cap Magazine and the Main Cap Channel. Shows placement of beam break sensors (Black) and Linear Actuators."
  - image_path: /assets/images/hedgehog/final.jpg
    excerpt: "Zoomed in Assembly of the First Stage, showing the Pick Up Platform."
---

## Introduction
  This was a consultation project for a YC22 agri-tech startup through Berkeley Engineering Solutions, a mechanical engineering consulting organization I co-founded. The start-up is focused on automating mushroom farming, and the first step in this process is to pauestrize the mushroom feed to elminate all possible 

## Design Considerations
- Can load and operate with 25kg of substrate
- Has to have a closed system of air that is contamination proof
- Can reach all necessary temperatures for a complete pasteurization cycle
  - Room temp to 70C
- Has to operate automatically
  - Must be able to run a full cycle with < 5min of human interaction per day per machine 
- Must sense compost temperature, air oxygen content, volumetric air flow, and inlet air temp.
  - Data should be logged and saved for every trial
  - Data collection rate should meet or exceed 4 samples per minute
- Temperature gradient across substrate must be < 2 deg C throughout cycle
- Must be easy to clean and can be completely sterilized between cycles
  - Clean time should be < 15min per cycle
- Air flow rate must be adjustable and controllable
  - Must be able to reach 500 m^3/hr/ton substrate at maximum
    - 13.5m^3/hr or 8cfm


## Inital Concepts
{% include figure image_path="/assets/images/hedgehog/concepts1.png" alt="" caption="" %}
{% include figure image_path="/assets/images/hedgehog/concepts3.png" alt="" caption="" %}
{% include figure image_path="/assets/images/hedgehog/cad-iso.png" alt="" caption="" %}
{% include figure image_path="/assets/images/hedgehog/cad-section.png" alt="" caption="" %}

## Prototype & Testing
{% include feature_row %}
{% include feature_row id= "feature_row2" %}



## Final Design
{% include feature_row id= "feature_row3" type="center" %}


## Successes and Failures
Me and my team were able to run our oven to successfully pausterize the composts in our first and only test with compost. However, due to an inabilty to properly tune the PID loops that ensured the oven successfully hit its temperature targets, the oven hit the first target temperature and held it for duration of the test.
