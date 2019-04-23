function flow = computeFlow(ptkey, ptframe)
    % build transformed frame and compute flow
    t1 = (ptkey - ptframe).^2;
    t2 = sum(t1, 2);
    t3 = sum(sqrt(t2));
    flow = t3 / size(ptkey, 1);
end