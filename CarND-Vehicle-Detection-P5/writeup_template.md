##Writeup Template
###You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

###File included in this submission
The following files have been included in this submission
* Ipython notebook with the code - P5_vehicle_detection_final.ipynb
* Folder with output images
* Ouput video with lane lines identified
* This write up file

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. 

You're reading it!

###Histogram of Oriented Gradients (HOG)

####1. Explain how (and identify where in your code) you extracted HOG features from the training images.

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:
<img src="./output_images/carandnotcar.png" width="600">

The dataset used contained 2826 cars and 8968 not car images. This dataset is unbalanced. I decided to leave it unbalanced since in the project video not car images far exceed the car images. The code for this step is contained in the code cell **2** of the IPython notebook.

I started by exploring the color features - spatial binning and color histogram. For spatial binning, I reduced the image size to 16,16 and the plot below shows the difference in spatial binning features between car and notcar images for channel - RGB. The plot delta shows the difference b/w car and notcar features

<img src="./output_images/binning_RGB.png" width="600">
The code for this step is contained in the code cell **3 and 6** of the IPython notebook. In the end I decided to not use color features (histogram and spatial binning) as it adversely affected performance.

Next I looked at HOG features using skimage.hog() functions. The key parameters are 'orientations', 'pixels_per_cell' and 'cells_per_block'. The num of orientations is the number of gradient directions. The pixels_per_cell parameter specifies the cell size over which each gradient histogram is computed. The cells_per_block parameter specifies the local area over which the histogram counts in a given cell will be normalized. To get a feel for the affect of  pixels_per_cell and cells_per_block, I looked at hog images with different settings. All the images below are from gray scale. The code for this step is contained in the code cell **4** of the IPython notebook.
<img src="./output_images/hog1.png" width="600">
<img src="./output_images/hog2.png" width="600">
<img src="./output_images/hog3.png" width="600">

####2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters and finally settled on the choice:
color_space = 'YCrCb' - YCrCb resulted in far better performance than RGB, HSV and HLS
orient = 9  # HOG orientations - I tried 6,9 and 12. Model performance didn't vary much
pix_per_cell = 8 - I tried 8 and 16 and finally chose 8
cell_per_block = 1 - I tried 1 and 2. The performance difference b/w the 2 wasn't much but 1 cell per block had significantly less no. of features and speeded up training and pipeline
hog_channel = 'ALL' -  ALL resulted in far better performance than any other individual channel

I spent a lot of choice narrowing down on these parameters. In the beginning I relied on the test accuracy in SVM classifier to choose parameters but then found that this wasn't indicative of performance in the video.

####3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

I trained a linear SVM using...

###Sliding Window Search

####1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I decided to search random window positions at random scales all over the image and came up with this (ok just kidding I didn't actually ;):

![alt text][image3]

####2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched on two scales using YCrCb 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result.  Here are some example images:

![alt text][image4]
---

### Video Implementation

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./project_video.mp4)


####2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Here's an example result showing the heatmap from a series of frames of video, the result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:

### Here are six frames and their corresponding heatmaps:

![alt text][image5]

### Here is the output of `scipy.ndimage.measurements.label()` on the integrated heatmap from all six frames:
![alt text][image6]

### Here the resulting bounding boxes are drawn onto the last frame in the series:
![alt text][image7]



---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

