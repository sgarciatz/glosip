# GLOSIP

Global Latency Optimization Single instance Placement (GLOSIP) contains the solver to address the problem proposed in the "Optimizing IoT Microservices Placement for latency reduction in UAV-Assisted Wireless Networks" paper.

If you are going to use this code or want to reference the paper, please use the following:

    @INPROCEEDINGS{10298356,
      author={García-Gil, Santiago and Murillo, Juan Manuel and Galán-Jiménez, Jaime},
      booktitle={2023 IEEE 20th International Conference on Mobile Ad Hoc and Smart Systems (MASS)}, 
      title={Optimizing IoT Microservices Placement for latency reduction in UAV-Assisted Wireless Networks}, 
      year={2023},
      volume={},
      number={},
      pages={658-663},
      keywords={Cellular networks;5G mobile communication;Wireless networks;Microservice architectures;Quality of service;Machine learning;Programming;UAV;IoT;latency;microservice;MILP},
      doi={10.1109/MASS58611.2023.00093}}

# How to run it?
Clone the repo:

    git clone https://github.com/sgarciatz/glosip.git

Create and start a Python virtual environment inside the cloned repo:

    python3 -m venv .venv
    source .venv/bin/activate

Install dependencies:

    pip install -r requirements.txt

Execute the solver:

    python GLOSIP.py

# Exporting to Network-Katharizer
After the execution of the GLOSIP.py file, an output file will be generated. That file can be used within the Network-Katharizer.
