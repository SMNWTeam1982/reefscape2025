# reefscape2024
Reefscape FRC Competition Team1982 project code

# Dependency Management
This project is using pipenv to manage dependencies, to install pipenv, use
```
pip install --user pipenv
```
This will install pipenv globally, which can be used with 
```
pipenv install <package>
```
To run commands in the virtual environment, use:
```
pipenv run python3 <command>
```
For example, to run the `robot.py` file, use:
```
pipenv run python3 robot.py
```
To download the RoboRIO precompiled binaries, use:
```
pipenv run sync
```
You may want to configure your editor (VSCode) to use the virtual environment that pipenv sets up, which can be done by opening the command palette and entering 'Python: Select Interpreter' and choosing the virtual environment corresponding to the output of `pipenv --venv`

#### Note:
Please do not edit `Pipfile.lock`, as this file is managed by pipenv and can break otherwise.

### PS:
Here is Ishant's super summarized notes of the wpilib documentation [here](https://docs.google.com/document/d/1EDiLUcMfb8uSZMkvbeYRN-dqZSBR8DbOoTo5ZQGXbvk/edit?usp=sharing)
