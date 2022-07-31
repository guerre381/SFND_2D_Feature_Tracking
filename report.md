# MiD-Term Report

# Data Buffer
- We use a vetor with maximum capacity equal to buffer size. 
 The first value in position 0 is deleted when the rest of the values are shifted to the left. 
  Since the last positon on the right extremity is left empty, we place the new frame there. 

# Keypoints
- keypoints detector is selectable via the string detectorType: {HARRIS, FAST, BRISK, ORB, SIFT, AKAZE}
 
- Irrelevant keypoints are removed thanks to a filter provided by openCV that needs a mask image.
  The mask image set to 1 where is the preceding vehicle and 0 everywhere else.

# Descriptors
- keypoints descriptor is selectable via the string descriptorType: {BRIEF, ORB, FREAK, AKAZE, SIFT}

- matching algorithm is selectable via the string matcherType: {MAT_BF, MAT_FLANN}
- actual operaction is selectable via the string selectorType: {SEL_NN, SEL_KNN}
  in case of KNN we compare the distance of the best 2 matches for each keypoints. if ratio is , 
  the description ogf that point is rejected when the similarity/ratio is higher than 0.8.

# Performance

- In the folloowing table we compare the performace for pairwise combination of detectors and descriptors.
  There are empty cells because the combination is simply not possibl or processing time was too long in case of HARRIS detector.

- For the analysis, we keep track of the following 3 metrics:
  - k is the number of keypoints detected.
  - m is the number of matches
  - t is the overall time for detection and description of keypoints


| detector/descriptor | SIFT                  | BRIEF                 | FREAK                 | AKAZE                 | ORB                   |
|---------------------|-----------------------|-----------------------|-----------------------|-----------------------|-----------------------|
| SIFT                | k=138, m=138, t=268.6 | k=138, m=138, t=195.9 | k=138, m=138, t=246.5 |                       |                       |
| HARRIS              |                       |                       |                       |                       |                       |
| FAST                | k=408, m=410, t=86.6  | k=408, m=410, t=20,7  | k=408, m=410, t=73.8  |                       | k=408, m=410, t=18.3  |
| BRISK               | k=277, m=278, t=570.0 | k=277, m=278, t=498.3 | k=277, m=258, t=546.2 |                       | k=277, m=278, t=497.6 |
| ORB                 | k=118, m=115, t=104.4 | k=118, m=115, t=22.0  | k=118, m=64, t=74.1   |                       | k=118, m=115, t=24.5  |
| AKAZE               | k=167, m=165, t=164.2 | k=167, m=165, t=147.1 | k=167, m=165, t=199.3 | k=167, m=165, t=229.8 | k=167, m=165, t=140.2 |

- A good combination is supposed to detect a lot of keypoint and match ups in  a short time. Time is really decisive.
- The top 3 combinations are:
  1- FAST and ORB.
  2- FAST and BRIEF.
  3- ORB and BRIEF. 