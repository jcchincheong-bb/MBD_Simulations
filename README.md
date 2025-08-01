# Planar Multibody Dynamics Simulations
A general purpose simulator for the kinematics and dynamics of two dimensional multibody systems. 

Heavily inspired by the work of Javad N. Nikravesh, **Planar Multibody Dynamics**. I started it as part of a course on multibody dynamics in my university Hoschule Rhein-Waal, but I have since continuously improved it. Unlike Nikravhesh, this project uses the MATLAB symbolic toolbox to generalise formulations and also increases the accuracy of calculations at a marginal decrease in speed. 

## Current Content
- Essential functions for dynamics using body coordinate formulation (see Functions folder)
- General purpose scripts to simulate multibody systems using body coordinate formulation (see BC_Formulation folder)\
- Two example mechanisms from the text by Nikravesh.
<img src='G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\Mechanisms\DoubleArmSuspension(DAS)\DAS.png' width='400'>
<img src='G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\Mechanisms\McPhersonStrut(MPS)\MPS.png' width='400'>
## Requirements
- MATLAB R2018b or later
- Symbolic Math Toolbox (optional, for symbolic derivations)
## Future Work
- I am working to implement kinetic analyses like forward and reverse dynamics
  
## References
Nikravesh, J. N. (2007). Planar Multibody Dynamics: Formulation, Programming with MATLAB®, and Applications. CRC Press.

## Contributing
Contributions are welcome! Feel free to submit a pull request or open an issue to suggest improvements or add new mechanism examples.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

Disclaimer: This code is for educational and research purposes only. It is not intended for use in industry or any commercial purposes.
