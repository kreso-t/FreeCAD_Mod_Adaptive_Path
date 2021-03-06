# FreeCAD Add-on - Adaptive Path Operation

## Description
Adaptive Path Operation - add-on to Path workbench that generates toolpaths while keeping the constant material removal rate throughout the cut

## Installation
1. Prerequisite - install the pyclipper library into the FreeCAD's python environment, i.e.
    ```bash
        pip install pyclipper
    ````

2. Clone the FreeCAD_Mod_Adaptive_Path source into to ~/.FreeCAD/Mod/FreeCAD_Mod_Adaptive_Path
3. Start the FreeCAD and add the "Adaptive 2.5D" command to the toolbar - the command will not automatically appear in the toolbar, you need to add it by customizing the Path toolbar

    ![add custom toolbar](./Resources/Inst2.png)

4. You are ready to go

## Usage
The usage of the Adaptive Path Operation is similar to other Path operations. You may select faces and either cut inside of the edges of selected face or outside (within the stock boundary). You also may select vertical faces to define cut region. By using the "Helix max diameter" you may decrease the helix (plunge) diameter in the narrow areas if necessary. "Lift distance" defines distance by which to raise the tool in the repositioning (G0) moves when there are no obstacles in between (if there are obstacles the tool is raised to Safe Height). "Step Over" and "Step Down" define the material removal rate that will algorithm try to keep constant during the cut.

## Screenshots

  ![ss1](./Resources/example1.png)

  ![ss2](./Resources/example2.png)

  ![ss3](./Resources/example3.png)

