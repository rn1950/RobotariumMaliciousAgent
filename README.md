# RobotariumMaliciousAgent
Tracking a malicious agent in the Georgia Tech ECE Robotarium. Code found in SUSD_3AGENT_MOVINGSOURCE.m

In this expirament, I implemented the SUSD (Speed Up Slow Down) strategy for source seeking where the source is a malicious agent. This strategy is described by Said Al-Abri in his paper: A Gradient-Free Three-Dimensional Source Seeking Strategy With Robustness Analysis found at https://ieeexplore.ieee.org/abstract/document/854044. A quadratic field was calculated with the malicious agent as the origin and these field values were used for the SUSD calculations.

Thanks to Said Al-Abri of GT Systems Research lab for helping me debug and having existing algorithms such as calculating the Covariance Matrix and PCA that I was able to use.

Video of the expirament: https://www.youtube.com/watch?v=1vVE-Ey-m5c&t=108s
Video of the MATLAB simulation prior to the expirament: https://www.youtube.com/watch?v=oSsHElmA0o0&t=54s

This code was run in the Robotarium software structure found at: https://github.com/robotarium/robotarium-matlab-simulator