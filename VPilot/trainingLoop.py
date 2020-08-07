


# While true:
#   Download newest Model version from Cluster

#   Drive for x time
#       Evaluate model performance on generated frames
#           Use newest Model version to get loss in pytorch 
#       drive to most informative images
#
#       Search with a Grid/Beam search?
#       With a Gradient Search?
#   Train Model with new generated, informative images, to do this:
#       Upload new Data to Cluster
#       Start Training Job
#       OR: Have an active Training job with an open input pipeline, that is waiting for newly generated data (e.g. with important sampling, where newest images get the highest priority)  TODO: How does this interact with Mosaik?
#       