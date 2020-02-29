# Introduction
We build this simulator to study RetroI2V's performance in practical I2V scenarios. We use Missing Rate, a common performance metric for RFID systems, as the metric.
We consider three key parameters in practical scenarios which are uplink rate, road traffic and retrosign density.
# build
To make the simulator more friendly, we provide a shell script. You can run this script as following to get the result of evaluation.
		
	bash ./origin_test.sh
	
While the simulation finished, the result was generated and saved in `origin_test/`. We provide three scripts to figure out the final result.  
You can go to `EVAL/tag_density/` and run 
	
	python3 traffic_pattern.py ../../origin_test
to get the evaluation result of tag density. The evaluation results of uplink rate and road traffic can be got by the same way.


