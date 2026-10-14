jaka = importrobot('../jaka_description/urdf/dual_zu5.urdf'); 
% load('dual_zu5.mat','jaka');
iviz = interactiveRigidBodyTree(jaka);
iviz.ShowMarker = false;