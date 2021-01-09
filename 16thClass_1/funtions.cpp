#include "stdafx.h"
#include "funtions.h"


std::vector<std::string> classes;

float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
int inpWidth = 416;        // Width of network's input image
int inpHeight = 416;       // Height of network's input image


// key point 连接表, [model_id][pair_id][from/to]
// 详细解释见
// https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md

int POSE_PAIRS[3][20][2] = {
	{   // COCO body
		{ 1,2 },{ 1,5 },{ 2,3 },
		{ 3,4 },{ 5,6 },{ 6,7 },
		{ 1,8 },{ 8,9 },{ 9,10 },
		{ 1,11 },{ 11,12 },{ 12,13 },
		{ 1,0 },{ 0,14 },
		{ 14,16 },{ 0,15 },{ 15,17 }
	},
	{   // MPI body
		{ 0,1 },{ 1,2 },{ 2,3 },
		{ 3,4 },{ 1,5 },{ 5,6 },
		{ 6,7 },{ 1,14 },{ 14,8 },{ 8,9 },
		{ 9,10 },{ 14,11 },{ 11,12 },{ 12,13 }
	},
	{   // hand
		{ 0,1 },{ 1,2 },{ 2,3 },{ 3,4 },         // thumb
		{ 0,5 },{ 5,6 },{ 6,7 },{ 7,8 },         // pinkie
		{ 0,9 },{ 9,10 },{ 10,11 },{ 11,12 },    // middle
		{ 0,13 },{ 13,14 },{ 14,15 },{ 15,16 },  // ring
		{ 0,17 },{ 17,18 },{ 18,19 },{ 19,20 }   // small
	} };

std::vector<cv::String> getOutputsNames(const cv::dnn::Net& net)
{
	static std::vector<cv::String> names;
	if (names.empty())
	{
		//Get the indices of the output layers, i.e. the layers with unconnected outputs
		std::vector<int> outLayers = net.getUnconnectedOutLayers();

		//get the names of all the layers in the network
		std::vector<cv::String> layersNames = net.getLayerNames();

		// Get the names of the output layers in names
		names.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
			names[i] = layersNames[outLayers[i] - 1];
	}
	return names;
}

//非极大值抑制，去除置信度较小的检测结果
void postprocess(cv::Mat& frame, std::vector<cv::Mat>& outs)
{
	std::vector<int> classIds;
	std::vector<float> confidences;
	std::vector<cv::Rect> boxes;

	for (size_t i = 0; i < outs.size(); ++i)
	{
		// Scan through all the bounding boxes output from the network and keep only the
		// ones with high confidence scores. Assign the box's class label as the class
		// with the highest score for the box.
		//
		float* data = (float*)outs[i].data;
		for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
		{
			cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
			cv::Point classIdPoint;
			double confidence;

			//获得得分最高的结果的分值和位置
			cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);

			if (confidence > confThreshold)
			{
				int centerX = (int)(data[0] * frame.cols);
				int centerY = (int)(data[1] * frame.rows);
				int width = (int)(data[2] * frame.cols);
				int height = (int)(data[3] * frame.rows);
				int left = centerX - width / 2;
				int top = centerY - height / 2;

				classIds.push_back(classIdPoint.x);
				confidences.push_back((float)confidence);
				boxes.push_back(cv::Rect(left, top, width, height));
			}
		}
	}

	//非极大值抑制，去除置信度较小的检测结果
	std::vector<int> indices;
	cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		cv::Rect box = boxes[idx];
		drawPred(classIds[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, frame);
	}
}

//检测结果绘制
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
	//绘制检测框
	cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255));

	//获得识别结果的类名称，以及置信度
	std::string label = cv::format("%.2f", conf);
	if (!classes.empty())
	{
		CV_Assert(classId < (int)classes.size());
		label = classes[classId] + ":" + label;
	}
	else
	{
		std::cout << "classes is empty..." << std::endl;
	}

	//绘制标签
	int baseLine;
	cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseLine);
	top = std::max(top, labelSize.height);
	cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
}