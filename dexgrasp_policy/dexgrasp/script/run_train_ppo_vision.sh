# CUDA_VISIBLE_DEVICES=3,0,1,2,4,5,6,7 \
CUDA_VISIBLE_DEVICES=0 \
python train.py \
--task=ShadowHandRandomLoadVision \
--algo=ppo \
--seed=0 \
--rl_device=cuda:0 \
--sim_device=cuda:0 \
--logdir=logs/test \
--headless \
--vision \
--backbone_type pn #pn/transpn 
#--test