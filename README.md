
# RetroI2V 
RetroI2V is a novel infrastructure-to-vehicle (I2V) communication and networking system that renovates conventional road signs to convey additional and dynamic information to vehicles while keeping intact their original functionality.
RetroI2V exploits the retroreflective coating of road signs and establishes visible light backscattering communication (VLBC), and further coordinates multiple concurrent VLBC sessions among road signs and approaching vehicles.
RetroI2V features a suite of novel VLBC designs including late-polarization, complementary optical signaling and polarization-based differential reception which are crucial to avoid flickering and achieve long VLBC range, as well as a decentralized MAC protocol that make practical multiple access in highly mobile and transient I2V settings. Experimental results from our prototyped system show that \sys achieves 5.3 dB SNR improvement over the status-quo and up to 101 $m$ communication range (with 1\% BER), and supports multiple access at scale.  
# Simulator Introduction
We build this simulator to study RetroI2V's performance in practical I2V scenarios. We use Missing Rate, a common performance metric for RFID systems, as the metric.
We consider three key parameters in practical scenarios which are uplink rate, road traffic and retrosign density.
# How to build?
To make the simulator more friendly, we provide a shell script. You can run this script as following to get the result of evaluation.
        
    bash ./origin_test.sh
    
While the simulation finished, the result was generated and saved in `origin_test/`. We provide three scripts to figure out the final result.  
You can go to `EVAL/tag_density/` and run 
    
    python3 traffic_pattern.py ../../origin_test
to get the evaluation result of tag density. The evaluation results of uplink rate and road traffic can be got by the same way.
# Publication
Purui wang, Lilei Feng, Guojun Chen, Chenren Xu, Yue Wu, Kenuo Xu, Guobin Shen, Kuntai Du, Gang Huang, Xuanzhe Liu

ACM MobiCom 2020 [[pdf](http://soar.group/pubs/RetroI2V.MobiCom20.pdf)]
