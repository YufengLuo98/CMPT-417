# CMPT-417, Implementation of CBS with disjoint splitting

Taeyoung Eun, Yufeng Luo, Derrick Cham

## Getting Started

### Prerequisites

These commands have been run on MacOS 10.14.6, it is required to use Python 3.8+ installed on your machine.

## Running tests on our map instances

Map instances are located in 'example_instances/'

A batch test can be performed with ``` python run_experiments.py --instance "example_instances/instance_*" --solver CBS --batch ```

or ran individually with ``` python run_experiments.py --instance "example_instances/test_*" --solver CBS ``` with the * replaced by a number 0-9

### Resulting Data

Our resulting data for CBS and ICTS can be seen below.

Increasing Cost Tree Search source code can be found here ``` https://github.com/AdamBignell/ICTS-vs-EPEA ```

![Screen Shot 2021-04-25 at 6 38 36 PM](https://user-images.githubusercontent.com/72104740/116017835-86ad3180-a5f5-11eb-952f-bd1757518096.png)

![Screen Shot 2021-04-25 at 6 38 43 PM](https://user-images.githubusercontent.com/72104740/116017842-8c0a7c00-a5f5-11eb-94cb-61b34244c3cf.png)
