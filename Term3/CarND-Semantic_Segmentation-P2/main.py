import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    with sess.as_default():
        vgg_tag = 'vgg16'
        model = tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)

        vgg_input_tensor_name = tf.get_default_graph().get_tensor_by_name('image_input:0')
        vgg_keep_prob_tensor_name = tf.get_default_graph().get_tensor_by_name('keep_prob:0')
        vgg_layer3_out_tensor_name = tf.get_default_graph().get_tensor_by_name('layer3_out:0')
        vgg_layer4_out_tensor_name = tf.get_default_graph().get_tensor_by_name('layer4_out:0')
        vgg_layer7_out_tensor_name = tf.get_default_graph().get_tensor_by_name('layer7_out:0')
    
    return vgg_input_tensor_name, vgg_keep_prob_tensor_name, vgg_layer3_out_tensor_name, vgg_layer4_out_tensor_name, vgg_layer7_out_tensor_name

tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function
     # First add a 1x1 convolution layer to the last layer of vgg
    # Set Kernel Size= 1 and Stride =1 for 1x1 convolution
    conv_1x1 = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, 1, kernel_initializer=tf.random_normal_initializer(stddev=0.01))

    #Add deconvolution layer
    # Set kernel size = 4,4 and Stride = 2,2 to upsample
    deconv_1 = tf.layers.conv2d_transpose(conv_1x1, num_classes, (4, 4), (2, 2), padding = 'SAME', kernel_initializer=tf.random_normal_initializer(stddev=0.01))

    # Connect Skip layer to Pool 4 from VGG
    conv_pool4 = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, 1, kernel_initializer=tf.random_normal_initializer(stddev=0.01))
    skip_layer1 = tf.add(deconv_1,conv_pool4)

    # Add second deconvolution layer with kernel size - 4,4 and stride- 2,2 and connect to the skip layer
    deconv_2 = tf.layers.conv2d_transpose(skip_layer1, num_classes, (4, 4), (2, 2), padding='SAME', kernel_initializer=tf.random_normal_initializer(stddev=0.01))
    conv_pool3 = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, 1, kernel_initializer=tf.random_normal_initializer(stddev=0.01))
    skip_layer2 = tf.add(deconv_2, conv_pool3)

    #Final deconvolution layer - kernel size 16,16 and stride - 8,8
    deconv_3 = tf.layers.conv2d_transpose(skip_layer2, num_classes, (16,16), strides=(8, 8), padding='SAME', kernel_initializer=tf.random_normal_initializer(stddev=0.01))

    return deconv_3
tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # Reshape logits and labels
    logits = tf.reshape(nn_last_layer, (-1, num_classes))
    labels = tf.reshape(correct_label, (-1, num_classes))
    #Calculate cross entropy loss
    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=labels))
    loss_operation = tf.reduce_mean(cross_entropy_loss)
    #Optimize with Adam Classifier
    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
    training_operation = optimizer.minimize(loss_operation)
    return logits, training_operation, cross_entropy_loss
tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    # Keep Prob and Learning Rate passed to the model
    for epoch in range(epochs):
        for image, image_label in get_batches_fn(batch_size):
            _, loss = sess.run([train_op, cross_entropy_loss],feed_dict = {input_image: image, correct_label: image_label, keep_prob: 0.5, learning_rate: 0.001})
        print("Epoch {} ".format(epoch+1), "out of {} -".format(epochs), " Loss: {:.6f}".format(loss))

tests.test_train_nn(train_nn)


def run():
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)
    #learning_rate = 0.001
    
    epochs = 45
    batch_size = 20
   
  

    # Download pretrained vgg model
    #helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # TODO: Build NN using load_vgg, layers, and optimize function
        image_input, keep_prob_tensor, layer_3_out, layer_4_out, layer_7_out = load_vgg(sess, vgg_path)
       
        nn_last_layer = layers(layer_3_out, layer_4_out, layer_7_out, num_classes)
        correct_label = tf.placeholder(dtype=tf.float32, shape=(None, None, None, num_classes))
        learning_rate = tf.placeholder(dtype=tf.float32)

        logits, training_operation, cross_entropy_loss = optimize(nn_last_layer, correct_label, learning_rate, num_classes)

        # TODO: Train NN using the train_nn function
        sess.run(tf.global_variables_initializer())
        train_nn(sess, epochs, batch_size, get_batches_fn, training_operation, cross_entropy_loss, image_input,
                 correct_label, keep_prob_tensor, learning_rate)

        # TODO: Save inference data using helper.save_inference_samples
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob_tensor, image_input)

        # Save model weights to disk
        saver = tf.train.Saver()
        saver.save(sess, 'my-model')
        print("Model saved in file")


        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()
