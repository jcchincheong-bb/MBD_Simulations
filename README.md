# Planar Multibody Dynamics Simulations
[Project Page](https://jcchincheong-bb.github.io/projects/mbd-simulations/index/)
A general purpose simulator for the kinematics and dynamics of two dimensional multibody systems. 

Heavily inspired by the work of Javad N. Nikravesh, **Planar Multibody Dynamics**. I started it as part of a course on multibody dynamics in my university Hoschule Rhein-Waal, but I have since continuously improved it. Unlike Nikravhesh, this project uses the MATLAB symbolic toolbox to generalise formulations and also increases the accuracy of calculations at a marginal decrease in speed. 

## 🔧 Current Features

- Core functions for dynamics using the **body coordinate formulation**  
  *(see the `Functions/` directory)*
- General-purpose scripts for simulating multibody systems  
  *(see the `BC_Formulation/` directory)*
- Example simulations of two mechanisms from Nikravesh's text

<p float="left">
  <img src='https://github.com/jcchincheong-bb/MBD_Simulations/blob/main/src/assets/DAS.png' width='400'>
  <img src='https://github.com/jcchincheong-bb/MBD_Simulations/blob/main/src/assets/MPS.png' height='200'>
</p>

## 📦 Requirements
- MATLAB R2018b or later
- Symbolic Math Toolbox 

## 🚧 Future Work
- Implementation of kinetic analyses (e.g., forward and inverse dynamics)
- Additional examples and validation cases
- More Animations
- Generalising paths for mechanism examples (right now they only work for the path on my computer)
  
## 📚 References
Nikravesh, J. N. (2007). Planar Multibody Dynamics: Formulation, Programming with MATLAB®, and Applications. CRC Press.

<img src='https://github.com/jcchincheong-bb/MBD_Simulations/blob/main/src/assets/Nikravesh.jpg' width='200'>

## 🤝 Contributing
Contributions are welcome! Feel free to submit a pull request or open an issue to suggest improvements or add new mechanism examples.

## 📄 License
This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.

> **Disclaimer:** This project is intended for educational and research purposes only. It is not suitable for industrial or commercial use without further validation.
