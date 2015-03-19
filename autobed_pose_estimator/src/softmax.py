# This piece of software is bound by The MIT License (MIT)
# Copyright (c) 2014 Siddharth Agrawal
# Code written by : Siddharth Agrawal
# Email ID : siddharth.950@gmail.com

import struct
import numpy
import array
import time
import scipy.sparse
import scipy.optimize
import pickle as pkl
###########################################################################################
""" The Softmax Regression class """

class SoftmaxRegression(object):

    #######################################################################################
    """ Initialization of Regressor object """

    def __init__(self, input_size, num_classes, lamda):
    
        """ Initialize parameters of the Regressor object """
    
        self.input_size  = input_size  # input vector size
        self.num_classes = num_classes # number of classes
        self.lamda       = lamda       # weight decay parameter
        
        """ Randomly initialize the class weights """
        
        rand = numpy.random.RandomState(int(time.time()))
        
        self.theta = 0.005 * numpy.asarray(rand.normal(size = (num_classes*input_size, 1)))
    
    #######################################################################################
    """ Returns the groundtruth matrix for a set of labels """
        
    def getGroundTruth(self, labels):
    
        """ Prepare data needed to construct groundtruth matrix """
        labels = numpy.array(labels).flatten()
        data   = numpy.ones(len(labels))
        indptr = numpy.arange(len(labels)+1)
        """ Compute the groundtruth matrix and return """
        ground_truth = scipy.sparse.csr_matrix((data, labels, indptr))
        gt_dense = ground_truth.todense()
        ground_truth = numpy.transpose(gt_dense[:, 1:(self.num_classes+1)])
        
        return ground_truth
        
    #######################################################################################
    """ Returns the cost and gradient of 'theta' at a particular 'theta' """
        
    def softmaxCost(self, theta, input, labels):
    
        """ Compute the groundtruth matrix """
    
        ground_truth = self.getGroundTruth(labels)
        
        """ Reshape 'theta' for ease of computation """
        
        theta = theta.reshape(self.num_classes, self.input_size)
        
        """ Compute the class probabilities for each example """
        
        theta_x       = numpy.dot(theta, input)
        hypothesis    = numpy.exp(theta_x)      
        probabilities = hypothesis / numpy.sum(hypothesis, axis = 0)
        
        """ Compute the traditional cost term """
        cost_examples    = numpy.multiply(ground_truth, numpy.log(probabilities))
        traditional_cost = -(numpy.sum(cost_examples) / input.shape[1])
        
        """ Compute the weight decay term """
        
        theta_squared = numpy.multiply(theta, theta)
        weight_decay  = 0.5 * self.lamda * numpy.sum(theta_squared)
        
        """ Add both terms to get the cost """
        
        cost = traditional_cost + weight_decay
        
        """ Compute and unroll 'theta' gradient """
        
        theta_grad = -numpy.dot(ground_truth - probabilities, numpy.transpose(input))
        theta_grad = theta_grad / input.shape[1] + self.lamda * theta
        theta_grad = numpy.array(theta_grad)
        theta_grad = theta_grad.flatten()
        
        return [cost, theta_grad]
    
    #######################################################################################
    """ Returns predicted classes for a set of inputs """
            
    def softmaxPredict(self, theta, input):
    
        """ Reshape 'theta' for ease of computation """
    
        theta = theta.reshape(self.num_classes, self.input_size)
        
        """ Compute the class probabilities for each example """
        
        theta_x       = numpy.dot(theta, input)
        hypothesis    = numpy.exp(theta_x)      
        probabilities = hypothesis / numpy.sum(hypothesis, axis = 0)
        
        """ Give the predictions based on probability values """
        
        predictions = numpy.zeros((input.shape[1], 1))
        predictions[:, 0] = numpy.argmax(probabilities, axis = 0)
        predictions += numpy.ones(numpy.shape(predictions))
        
        return predictions

###########################################################################################
""" Loads the images from the provided file name """

def loadImages(file_name):

    """ Open the file """

    image_file = pkl.load(open(file_name, 'rb'))
    
    """ Format the header information for useful data """
    
    num_examples = len(image_file)
    """ Initialize dataset as array of zeros """
    
    dataset = numpy.zeros((numpy.size(image_file, 1), num_examples))
    
    
    """ Normalize and return the dataset """    
    for i in range(num_examples):
        dataset[:, i] = (numpy.asarray(image_file[i]) / numpy.max(image_file[i]))
    
            
    return dataset 

###########################################################################################
""" Loads the image labels from the provided file name """
    
def loadLabels(file_name):

    """ Open the file """

    labels_raw = pkl.load(open(file_name, 'rb'))
    
    """ Format the header information for useful data """
    
    num_examples = len(labels_raw)
    
    """ Initialize data labels as array of zeros """
    
    labels = numpy.zeros((num_examples, 1), dtype = numpy.int)
    
    
    """ Copy and return the label data """
    
    labels[:, 0] = labels_raw[:]
    
    return labels

###########################################################################################
""" Loads data, trains the model and predicts classes for test data """

def executeSoftmaxRegression():
    
    """ Initialize parameters of the Regressor """
    
    input_size     = 64*27    # input vector size
    num_classes    = 3     # number of classes
    lamda          = 0.0001 # weight decay parameter
    max_iterations = 5000    # number of optimization iterations
    
    """ Load MNIST training images and labels """
    
    training_data   = loadImages('../database/training_images.p')
    training_labels = loadLabels('../database/training_labels.p')
    
    """ Initialize Softmax Regressor with the above parameters """
    
    regressor = SoftmaxRegression(input_size, num_classes, lamda)
    """ Run the L-BFGS algorithm to get the optimal parameter values """
    
    opt_solution  = scipy.optimize.minimize(regressor.softmaxCost, regressor.theta, 
                                            args = (training_data, training_labels,), 
                                            method = 'L-BFGS-B', jac = True,
                                            options = {'maxiter':max_iterations})
    opt_theta     = opt_solution.x
    print opt_theta 
    """ Load MNIST test images and labels """
    
    test_data   = loadImages('../database/test_images.p')
    test_labels = loadLabels('../database/test_labels.p')
    
    """ Obtain predictions from the trained model """
    
    predictions = regressor.softmaxPredict(opt_theta, test_data)
    """ Print accuracy of the trained model """
    correct = test_labels[:, 0] == predictions[:, 0]
    print """Accuracy :""", (numpy.mean(correct)*100)
    
executeSoftmaxRegression()
