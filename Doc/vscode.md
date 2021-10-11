VSCode {#vscode}
=======

Introduction {#vscode_intro}
===
[Visual Studio Code](https://code.visualstudio.com/) is a powerful text editor developed by Microsoft. FEH has developed an extension for this editor that, along with some standalong tools, enables building for the Proteus.

Installation {#vscode_install}
===
Windows {#vscode_install_windows}
---

Mac {#vscode_install_mac}
---
First, download and install Visual Studio Code from [here](https://code.visualstudio.com/Download). For help, see the installation instructions [here](https://code.visualstudio.com/docs/setup/mac).

Second, obtain the FEH VSCode extension from the O:/ drive, which can be accessed from any Hitchcock computer. The extension is located at O:/FEHvscode/_INSTALLERS/feh-vscode-extension.vsix. Move this file using remote storage or a flash drive to your Mac computer. Launch VSCode and open the extensions tab as shown in the image below.

![](@ref vscode_extension.png)

Click the show more actions button and select "Install from VSIX" as shown in the image below.

![](@ref vscode_install_vsix.png)

Navigate to the FEH extension VSIX file and select "Install" as shown in the image below.

![](@ref vscode_install_vsix2.png)

A message will appear stating "Completed installating the extension FEH VSCode Extension.".

Third, obtain the FEH Proteus toolchain from the O:/ drive. The toolchain is located at O:/FEHvscode/_INSTALLERS/proteus_toolchain_mac.zip. Move this file using remote storage or a flash drive to your Mac computer. Unzip the folder by double clicking on it. Navigate into the folder that is unzipped, named proteus_toolchain_mac. Double click the install_toolchain.command file. A terminal will pop up; when prompted, enter your password. Once the terminal says [Process Complete], exit the terminal. The extracted files and .zip file can be deleted.

Usage {#vscode_usage}
===
Creating a project {#vscode_usage_creating}
---
Navigate to the FEH extension tab as shown in the image below.

![](@ref vscode_tab.png)

Double click on the "Create new FEH Project" button. Navigate to the directory where you'd like to store your project, and select the "Select Project Location" button. Then, type the name of your project and press enter. This will open a new project in your explorer view, as shown in the image below.

![](@ref vscode_new_project.png)

Building a project {#vscode_usage_building}
---
