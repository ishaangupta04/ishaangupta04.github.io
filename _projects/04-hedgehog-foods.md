---
title: "Smart Oven"
excerpt: "Agri-Tech Smart Compost Oven"
order: 3
header:
  image: assets/images/hedgehog/final.jpg
  teaser: assets/images/hedgehog/teaser.jpg
sidebar:
  - title: "Role"
    image: 
    image_alt: 
    text: "Project Manager and Engineering Lead"
  - title: "Skills"
    text: "Project Mangement, BOM Mangement, Thermodynamics, CAD/CAM, PID Control, Manufacturing"

# Project Gallery
gallery:
  - url: /assets/images/hedgehog/proto-inside-body.jpg
    image_path: /assets/images/hedgehog/proto-inside-body.jpg
    alt: "Prototype interior showing heating element placement"
    title: "Body Interior Design"
  - url: /assets/images/hedgehog/proto-body1.jpg
    image_path: /assets/images/hedgehog/proto-body1.jpg
    alt: "Assembled prototype exterior view"
    title: "Prototype Assembly"
  - url: /assets/images/hedgehog/proto-motor.jpg
    image_path: /assets/images/hedgehog/proto-motor.jpg
    alt: "3D printed motor mounting bracket"
    title: "Custom Motor Mount"

# Final Implementation
feature_row2:
  - image_path: /assets/images/hedgehog/final-top.jpg
    alt: "Final product ready for testing"
    title: "Complete Assembly"
    excerpt: "Final product ready for test cycle"
    url: /assets/images/hedgehog/final-top.jpg
---

## Introduction
  This was a consultation project for a YC22 agri-tech startup through Berkeley Engineering Solutions, a mechanical engineering consulting organization I co-founded. The start-up that my team and I worked with is focused on automating mushroom farming, and the first step in this process is to pauestrize the mushroom feed to elminate all possible 

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
{% include figure image_path="/assets/images/hedgehog/cad-iso.png" alt="" caption="Full Assembly of the intial concept" %}
{% include figure image_path="/assets/images/hedgehog/cad-section.png" alt="" caption="Section View showing the interior. The feed is raised such that we can place a heating element bellow. Air will be intaked from the right, and will then be heated up within the box. It wil then rise through the box, heating up the compost in the process." %}

## Prototype Development
{% include gallery layout="half" %}

## Final Implementation
{% include figure image_path="/assets/images/hedgehog/final-top.jpg" alt="Final product ready for testing" caption="Complete assembly ready for test cycle" %}
{% include figure image_path="/assets/images/hedgehog/final.jpg" alt="Side view of control system" caption="Integrated control system showing motor controllers, intake fan, and microcontrollers" %}

## Reflection
Me and my team were able to run our oven to successfully pasteurize the composts in our first and only test with compost. However, due to an inability to properly tune the PID loops that ensured the oven successfully hit its temperature targets, the oven hit only the first target temperature and held it for the duration of the test instead of performing the multiple heating a cooling cycles it was meant to. Overall this was an immensely enjoyable project as it was the first project where I got the opportunity to apply my knowledge of thermodynamics and implement a PID controller in a more abstract application.

