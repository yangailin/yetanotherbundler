clear all;
clc;
load img000_H_mat.txt
load pt_match_between_000_001.txt

x1 = pt_match_between_000_001(:,1:2)';
x2 = pt_match_between_000_001(:,3:4)';

x1 = makehomogeneous(x1);

x2_hat = img000_H_mat*x1;
x2_hat = makeinhomogeneous(x2_hat);

err = x2_hat - x2;
plot(err(1,:),err(2,:),'.');