# RoboAnkle_DataProcessingToolbox
This directory contains all data processing scripts used during the robotic ankle-foot prosthesis experiments involving human participants. 

## Usage
- **cp_poweredAnkle_tui**: main script that can run continuously suring data collections. Follow the text-based commands once new data files are available for processing. 
- **cp_process_trial**: processes motion capture, force plate, and embedded system data for a single trial. 
- **cp_ilc**: data-driven iterative learning control utilizing adaptive learning gains and inverse transfer function model generation. 
- **cp_vicon_process_model**: processes motion capture marker model and inverse dynamics data files. 
- **cp_vicon_process_grf**: processes force plate data files. 
- **cp_labview_process_data**: processes logged embedded sensor and control command data from the [robotic prosthesis controller](https://github.com/chrisprasanna/RoboAnkle_Controller).
- **cp_vicon_process_subject**: processes subject-specific metadata. 
- **cp_Phi**: uses sampled force plate and thigh-mounted IMU data to compute offline phase variable calculations. 
