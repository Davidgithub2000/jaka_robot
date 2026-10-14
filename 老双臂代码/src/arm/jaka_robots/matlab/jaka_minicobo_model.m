jaka = importrobot('../jaka_description/urdf/jaka_minicobo.urdf'); 
% load('jaka_zu3.mat','jaka');
iviz = interactiveRigidBodyTree(jaka);
iviz.ShowMarker = false;