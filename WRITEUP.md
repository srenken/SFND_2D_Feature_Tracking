## MP.1 Data Buffer Optimization

- Max. dataBuffer size is defined by int var "dataBufferSize"
- While looping over each image I first print the current size of the dataBuffer (struct)
- Than I compare the size of the dataBuffer against the defined max. size "dataBufferSize" and if the max size haven't been reaach I push_back the current data frame (image) to the end of the dataBuffer vector

        std::cout << "Current DataBuffer Size: " << dataBuffer.size() << std::endl;

        // Fill ring buffer upto bufferSize
        if (dataBuffer.size() < dataBufferSize)
        {
            dataBuffer.push_back(frame);
        }

- In case the max. dataBufferSize is reached the first elemet of the dataBuffer vector will be erased and the new frame (image) pushed back (addded) at the end of the dataBuffer vector.

        // If buffer size is reached delete first element, than push_back new frame to the end
        else
        {
            dataBuffer.erase(dataBuffer.begin());
            dataBuffer.push_back(frame);
        }

- For debugging I print the bufferSize (vector) size after adding a new frame
        
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done -- new DataBuffer Size: " << dataBuffer.size() << endl;

--- 


## MP.2 Keypoint Detection

- The detector type can be selected by setting the string *detectorType* accordingly


        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = "FAST"; // -> SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

- Based on the set *detectorType* on of the following 3 functions is called
- The ShiTomasi Detector and Harris Detector is implement in it's own function
- All other detectors are implemented in *detKeypointsModern()* and need to have the *detectorType* passed as function parameter


        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, true);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, true);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, true);
        }



- Within the function *detKeypointsModern()* the *detectorType* string is compared against the possible detectors
- Foreach an opencv Pointer of the type of the detector class is created and instance of the according detector assign
- All detectors have the function *detect()* implemented which is called and the image as well as the reference (address)to keypoints vector is passed

```
    // Detect keypoints in image using Modern Detectors (FAST, BRISK, ORB, AKAZE, and SIFT)
    void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
    {
        // Timer Start
        double t = (double)cv::getTickCount();

        // Select Detector based on the set detectorType and perform detection
        if (detectorType.compare("SURF") == 0)
        {
            // Detect keypoints using SURF Detector
            int minHessian = 400;
            cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
            detector->detect( img, keypoints );
        }

        ..
        ..
    }
```
---

## MP.3 Keypoint Removal

- Next an rectangle of OpenCV class cv::Rect is defined with its width and height and x and y position. The size and position is defined to match the position of the vehicle in front
- This rectangle can than be used to check if the coordinates of each keypoint are within that rectangle
- If the keypoint is within the rectangle it is appended to a temporaly keypoints vector 
- After looping over all keypoints the original keypoint vector is replaced by the only the cropping keypoints, which are within the rectangle (temporaly keypoints vector)


```
        // only keep keypoints on the preceding vehicle

        cout << "-- Remove Keypoints outside the bounding box --" << endl;
        cout << "Num of Keypoints before cropping: " << keypoints.size() << endl;

        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        
        if (bFocusOnVehicle)
        {
            // temp keypoints vector to store cropped keypoints
            vector<cv::KeyPoint> keypoints_tmp;
        
            // Loop over all keypoints
            for (auto itr_kpts = keypoints.begin(); itr_kpts != keypoints.end(); ++itr_kpts)
            {
                // Crop keypoints if within the defined bounding box of the preceding vehicle
                if (vehicleRect.contains(itr_kpts->pt))
                {
                    keypoints_tmp.push_back(*itr_kpts);
                }
            }

            keypoints = keypoints_tmp;
        }
```

---

## MP.4 Keypoint Descriptors

- Keypoint detection for BRIEF, ORB, FREAK, AKAZE and SIFT is implemented in the function *descKeypoints()*
- Here again the descriptor can be selected by passing the acording *descriptorType* as a string 

```
cv::Mat descriptors;
string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType)
```

- In the function *descKeypoints()* first a generic DescriptorExtractor pointer is defined
- Than similar to the function *detKeypointsModern()* the descriptorType string is compared against the defined descriptor strings
- Foreach descriptor the default parameter are used and set, after a descriptorExtrator is created and assign to the exctrator pointer
- In the a last step the descriptors are computed by calling *extrator->compute()* 

```
// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // descriptorTypes: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }

    ...
    ...

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}
```

---

## MP.5 Descriptor Matching / MP.6 Descriptor Distance Ratio

- FLANN based descriptor matching, k-nearest neighbor selection and the descriptor distance ratio test is implemented in *matchDescriptors()*


- If the *matcherType* is set to the string "MAT_FLANN" a FLANN based descriptor Matcher is created by creating a DescriptorMatcher and passing *cv::DescriptorMatcher::FLANNBASED* as type
```
// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        // In case of Binary Descriptor use Hamming distance else L2_NORM (Sum of squared differences)
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2; 
       
        // Brute Force Matching
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F || descRef.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        // FLANN matching
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    ...
    ...
```

- In a similar way is the k-nearest neighbor selection and distance ratio test implemeted. It will be executed / selected if *selectorType* is set to the string "SEL_KNN"
- Than first a support vector *knnMatches*, which temporaly holds the 2 best matches for each descriptor
- Next the *knnMatch()* funtion of the previously created matcher is called. The last parameter sets k=2

- In the last step the distance ratio test is applied to all the knnMatch results and first of the 2 matches will only be added to the *matches* vector if the distance ratio test is succesfull (dist_1 < 0.8 * dist_2)

```
    ...
    ...

    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        vector< vector<cv::DMatch>> knnMatches;

        matcher->knnMatch(descSource, descRef, knnMatches, 2); // Finds the 2 best matches for each descriptor
  
        // descriptor distance ratio test

        float minDescDistRatio = 0.8;
        for(auto itr = knnMatches.begin(); itr != knnMatches.end(); ++itr)
        {
                        
            if (itr->at(0).distance < minDescDistRatio * itr->at(1).distance)
            {
                matches.push_back(itr->at(0));
            }
        }
    }

    ...
    ...


```
---

## MP.7 Performance Evaluation 1

see performance_evaluation.xlsx and logs

---

## MP.8 Performance Evaluation 2

see performance_evaluation.xlsx and logs

---

## MP.9 Performance Evaluation 3

Execution times are compare in performance_evaluation.xlsx and result in the following recommendation for the given purpose:

---

### TOP3: cetector / descriptor combinations:
* FAST - BRIEF
* FAST - BRISK
* FAST - ORB


Reason: Best executions times and it looks that there only a few (~0 - 3) mismatched keypoints per set of frames for these Detector / Descriptor combinations as well
