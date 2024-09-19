---
layout: default
math: true
---
#### [Nima Fazeli](https://www.mmintlab.com/people/nima-fazeli/)

<div style="text-align: right"> &copy; Nima Fazeli, 2024 </div>
<div style="text-align: right"> Last updated: 19-09-2024 </div>
<div style="text-align: right"> <a href="cite.md">How to cite</a>, How to give feedback </div>

<br/><br/>

**Note**: These are working notes used for [a course being taught at the University of Michigan](https://intro2manipulation.robotics.umich.edu/). They will be updated throughout the Fall 2024 semester. 

### Table of Contents

1. Mechanics of Rigid-Body Frictional Interaction
    1. [Preliminaries](mechanics/preliminaries.md)
        - Rigid-body Assumption
        - Non-penetration Assumption
        - States and Configuration Space
        - Contact Frames and Contact Planes
        - Force Transmissions and the Contact Jacobian
        - Contact Force Constraints (high-level)
        - Robot/Finger Jacobians
    2. [Grasp Analysis](mechanics/grasp-analysis.md)
        - Grasp Matrix
        - Form Closure
        - Force Closure
2. Planning and Controls for Rigid-Body Interaction
    1. [Introduction](planning-controls/preliminaries.md)
    2. [Dynamical Motion Primitives](planning-controls/dmps.md)
3. Perception for Rigid-Body Frictional Interactions


### Preface

Robotic manipulation is the science and art of robotic systems interacting with their physical environments. In particular, manipulation is the controlled and purposeful physical interaction of the robot with its environment to produce desired effects and changes. Manipulation is, in my very biased opinion, the most crucial function of robots. It encompasses a very wide array of tasks that enable robots to interact with and modify their surroundings. From delicate tasks such as picking up fragile objects to heavy-duty operations like assembling industrial components, robotic manipulation is the key that unlocks the potential for machines to perform tasks that were once limited to human dexterity and strength. Hopefully, robots of the future will surpass human capabilities and perform skills that lie well outside our reach.

I started my career in manipulation in 2014, a time before deep learning. The dominant paradigm in manipulation, and perhaps most other areas in robotics, was that of model-based perception and planning/controls. This often meant that we'd draw from contact mechanics to derive models that enabled robots to perform complex tasks such as pushing, pivoting, grasping, throwing, and many more. This approach has a long and rich history and shapes much of my thinking to this day. I find it deeply satisfying to try to understand how the physical world works and distill this understanding into models that enable robots to act gracefully and robustly. The *Mechanics* and *Planning* sections of these notes are reflections of this world view. These sections provide a foothold in the mountain knowledge that has accumulated over the years.

The impact of learning and AI in robotics is undeniable. It is likely that deep learning and AI is here to stay and the field is better for it. I believe that a fundamental understanding of contact is a key part of designing learning algorithms that can lead to graceful and robust behavior. Much like the role of convolutions for images or attention in modeling sequences, it is likely that we will see significant progress through judicious choices of representation architectures as well as perception and control algorithms. The *Perception* and *Learning* sections are geared towards a more modern taste in manipulation research. Much of the beginning sections of *Perception* treat classically algorithmic methods that likely served as some of the inspiration in more recent advances in learning. It is impractical to cover the rapidly expanding literature in learning, just as much as it is impractical to cover the vast wealth of knowledge we have of contact mechanics. Instead, these notes will attempt to provide some foundations that can serve as a starting point in navigating the tumultuous and rapdily changing seas of learning.

As a final note, manipulation is at the confluence of many disciplines: mechanics, multimodal perception, planning, controls, learning, embodied AI, mechanical design, sensing and sensors, human-robot interactions, communication protocols, edge computing,  ... . These notes are focused on a subset of these topics that specifically pretain to decision making and world models. There truly is something for everyone in manipulation research and I encourage you, the reader, to find what gives your inspiration wings and fly.

This course is greatly influenced by Prof. Matt Mason's [The Mechanics of Robotic Manipulation](https://direct.mit.edu/books/monograph/3869/Mechanics-of-Robotic-Manipulation) and Prof. Russ Tedrake [Robotic Manipulation](https://manipulation.csail.mit.edu/index.html). Please contact Prof. Nima Fazeli at __nfz@umich.edu__ for any feedback, suggestions, and corrections to these notes.
