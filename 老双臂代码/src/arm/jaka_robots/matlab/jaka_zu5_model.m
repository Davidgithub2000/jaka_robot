jaka = importrobot('../jaka_description/urdf/jaka_zu5.urdf'); 
% load('jaka_zu5.mat','jaka');
iviz = interactiveRigidBodyTree(jaka);
iviz.ShowMarker = false;