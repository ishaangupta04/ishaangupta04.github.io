---
title: "Automatic Capper"
excerpt: "An automated system for capping and decapping vials in laboratory environments"
order: 5
header:
  image: /assets/images/Capper/teaser_org.png
  teaser: /assets/images/Capper/teaser.png
sidebar:
  - title: "Role"
    text: "Lead Designer & Engineer"
  - title: "Skills"
    text: "Mechanical Design, Electrical Systems, Safety Compliance, Programming, PLC"
---

## Introduction

This project focused on developing an automated capper/decapper system for laboratory vials, building upon previous work on the cap dispenser project. The system was designed to interface with a UR-5 Robot Arm, which would introduce and remove sample caps from the system. The primary objectives were:

1. Refining the cap dispenser design for improved reliability and full deployment into the A-Lab system
2. Designing a new capper/decapper system from the ground up using an Ender 3 Pro as the base platform
3. Developing a lab safety-compliant electrical system
4. Sourcing and fabricating all necessary components

## Design Considerations

- Cap Alignment
  - Enable repeatable robot arm pick-ups and drop-offs
  - Ensure consistent capper/decapper operations
- Operation Speed
  - Improve upon existing capping/decapping speeds
  - Complete capping/decapping in a single operation
- Manufacturing & Assembly
  - 3D-printable body for rapid iteration
  - Reduced part count for simplified assembly
  - Reduced assembly time
- Platform Choice
  - Ender 3 Pro base platform with open-source firmware for simple programming
  - Rigid and reliable gantry to reduce design complexity
- Modularity
  - Fast repairs and independent iteration of components
  - System expandability for different volume requirements

## Mechanical System

{% include figure image_path="/assets/images/Capper/system-overview.png" caption="Figure 1: System Overview" %}

**Operation Process:**

**De-Capping Sequence:**
1. Upper gripper moves to home position
2. Robot arm places sample on lower gripper, which clamps the vial
3. Upper gripper clamps onto sample lid
4. Stepper motor rotates vial while gantry raises upper gripper
5. Sample unscrewed, lid placed on lid platform

**Capping Sequence:**
1. Upper gripper retrieves lid from platform
2. Stepper motor rotates vial while gantry lowers upper gripper
3. Sample capped and retrieved by robot arm

**Vial Gripper Assembly**
{% include figure image_path="/assets/images/Capper/vial-gripper.png" caption="Figure 2: Vial Gripper Assembly" %}
{% include figure image_path="/assets/images/Capper/gripper-cross-section.png" caption="Figure 3: Vial Gripper Assembly Cross Section" %}

The assembly features:
- SMC Pneumatic Gripper
- Rotary Union for air transmission through rotating joint
- Nema 23 motor providing up to 2 N*m torque
- Custom-designed components for integration

**Custom Components**

**Rotary Union Extension**
{% include figure image_path="/assets/images/Capper/rotary-union-extension.png" caption="Figure 4: Rotary Union Extension" %}

The Rotary Union Extension is attached to the 
Rotary Union by a shoulder bolt that passes 
through its center and threads into the Rotary 
Union. To constrain the 2 components 
rotationally, a pin is placed in the off center 
hole, and lines up with a similar hole on the 
Rotary Union itself. The power of the motor 
is transferred through the pin. This ensures 
that the shoulder bolt does not tighten and and 
loosen as the Stepper Motor turns clockwise 
and counterclockwise. Is currently 3-D 
printed, but should be machined out of steel to 
ensure longevity.



**Rotary Union Mount**
{% include figure image_path="/assets/images/Capper/rotary-mount.png" caption="Figure 5: Rotary Union Mount" %}
- Secures stationary half of Rotary Union
- Designed with access pockets for shaft coupler maintenance


**Lid Platform**
{% include figure image_path="/assets/images/Capper/lid-platform.png" caption="Figure 6: Lid Platform" %}
- Dual position design for removed and shaker lids
- Height-matched to Vial Gripper Assembly for optimized cycle times

## Pneumatic System

{% include figure image_path="/assets/images/Capper/manifold.png" caption="Figure 7: Manifold with two 5/3 solenoid valves" %}

{% include figure image_path="/assets/images/Capper/frl.png" caption="Figure 8: Filter Regulator Lubricator (FRL)" %}

**System flow:**
1. Wall air enters FRL filter for contaminant removal
2. Regulator maintains pressure within SMC Gripper range (0.1 to 0.6 MPa)
3. Lubricator adds oil particles for mechanical reliability
4. Electric shut-off solenoid controls system air flow
5. Manifold distributes air to 5/3 solenoids
6. Exhaust flow control valves manage gripper operation speed

## Electrical System

{% include figure image_path="/assets/images/Capper/electrical-box.jpg" caption="Figure 9: Electrical Box" %}
{% include figure image_path="/assets/images/Capper/plc.png" caption="Figure 10: ProductiveOpen Arduino Based PLC" %}

**Key Features:**
- Open-source solution using ProductiveOpen Controllers
- Arduino-based PLC system for enhanced maintainability
- DIN rail mounted components for safety compliance
- Remote control capability for Ender 3
- - BIGTREETECH SKR MINI E3 V2 motherboard upgrade for advanced control via Klipper

## Testing Demo
<video width="100%" controls>
  <source src="/assets/videos/Capper/Capper_Testing.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

