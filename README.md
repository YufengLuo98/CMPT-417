# CMPT-417, Implementation of CBS with disjoint splitting

Taeyoung Eun, Yufeng Luo, Derrick Cham

## Getting Started

### Prerequisites

These commands have been run on MacOS 10.14.6, it is required to use Python 3.8+ installed on your machine.

## Running tests on our map instances

Map instances are located in 'example_instances/'

A batch test can be performed with ``` python run_experiments.py --instance "example_instances/instance_*" --solver CBS --batch ```

or ran individually with ``` python run_experiments.py --instance "example_instances/test_*" --solver CBS ``` with the * replaced by a number 0-9
