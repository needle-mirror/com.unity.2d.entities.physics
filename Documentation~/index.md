# What is 2D Entities Physics?

2D Entities Physics is a package that provides tools to work with 2D physics in [Project Tiny](https://unity.com/solutions/instant-games). Project Tiny is Unity's upcoming highly modular runtime powered by [DOTS](https://unity.com/dots), that enables you to build [instant games](https://unity.com/solutions/instant-games) that are small in size and load and run quickly on mobile and web platforms.

For information regarding the system requirements, project setup guide and general information about Project Tiny, please refer to [Project Tiny: Getting Started](https://docs.google.com/document/d/1A8hen2hLFY5FLkC5gd3JP2Z-IpHfnAX-CpYLK3aOdwA) for the latest information.

## Currently supported features

The following features are partially supported by 2D Entities Physics. When working with DOTs, GameObjects located inside SubScenes are converted into their ECS counterparts. This converted data is used during runtime in the DOTs systems. However the properties contained in the components attached to the GameObjects are converted in this process. 

Please refer to the features’ respective sections for information about how their data is handled during the conversion to ECS:

- Primitive Colliders (namely [Box Collider 2D](Conversion.html#box-collider-2d), [Capsule Collider 2D](Conversion.html#capsule-collider-2d), [Circle Collider 2D](Conversion.html#circle-collider-2d), [Polygon Collider 2D](Conversion.html#polygon-collider-2d))
- [Compound Collider](Conversion.html#compound-collider)
- [Rigidbody 2D](Conversion.html#rigidbody-2d)
- Physics broadphase
- Broadphase querying API

## Requirements

This version of 2D Entities Physics is compatible with the following versions of the Unity Editor:

- 2020.1.0f1 and newer