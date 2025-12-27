---
title: U-Z Terms
sidebar_label: U-Z
---

# U-Z Terms

## Underfitting

**Definition**: A scenario where a machine learning model is unable to capture the underlying pattern of the data, usually due to insufficient complexity.

**Examples**:
- Linear model applied to non-linear data
- High bias in model predictions
- Poor performance on both training and test sets

## Validation Set

**Definition**: A subset of data used during training to tune hyperparameters and evaluate model performance during development.

**Examples**:
- Used for hyperparameter selection
- Helps detect overfitting during training
- Separate from training and test sets

## Weight Initialization

**Definition**: The process of setting initial values for the weights in a neural network before training begins.

**Examples**:
- Xavier/Glorot initialization
- He initialization for ReLU networks
- Random initialization strategies

## Zero-shot Learning

**Definition**: A machine learning paradigm where a model is expected to recognize and classify objects it has not seen during training.

**Examples**:
- Language models understanding new concepts
- Image classifiers recognizing new categories
- Cross-lingual transfer tasks

## Activation Map

**Definition**: A representation showing which neurons in a neural network are activated for a given input, often used in visualization.

**Examples**:
- Feature maps in CNNs
- Attention weights in transformer models
- Understanding model decisions

## Backpropagation Through Time (BPTT)

**Definition**: An extension of backpropagation for training recurrent neural networks by unfolding the network through time.

**Examples**:
- Training RNNs for sequence prediction
- Updating weights in LSTM networks
- Handling sequential data dependencies

## Curriculum Learning

**Definition**: A training approach that orders training examples from simple to complex, mimicking how humans learn.

**Examples**:
- Starting with easier classification tasks
- Gradually increasing difficulty of examples
- Improving convergence and performance

## Dropout

**Definition**: A regularization technique where randomly selected neurons are ignored during training to prevent overfitting.

**Examples**:
- Temporarily removing neurons during training
- Reducing co-adaptation of neurons
- Improving generalization