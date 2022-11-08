function randomWalk = generate2DRandomWalk(nSteps)
nRepeats = 1;
nDims = 2;
randomWalk = zeros(nSteps,nDims);
for i=1:nRepeats
   for j = 1:nSteps % Looping all values of nSteps into w_postion.
       x = sign(randn([1,nDims])); % Generates either +1/-1 depending on the sign of RAND.
       randomWalk(j+1,:) = randomWalk(j,:) + x;
   end
end
end