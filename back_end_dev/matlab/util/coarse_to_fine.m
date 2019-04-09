function DownSampImg = coarse_to_fine(SrcImg,rows,cols)
for y = 1:rows/2
    for x = 1:cols/2
        x0 = x * 2 - 1;
        x1 = x * 2;
        y0 = y * 2 - 1;
        y1 = y * 2;
        
        if length(size(SrcImg)) == 2
            a = SrcImg(y0,x0) / 4;
            b = SrcImg(y0,x1) / 4;
            c = SrcImg(y1,x0) / 4;
            d = SrcImg(y1,x1) / 4;
            DownSampImg(y,x) = a+b+c+d;
        else
            for k = 1:3
                a = SrcImg(y0,x0,k) / 4;
                b = SrcImg(y0,x1,k) / 4;
                c = SrcImg(y1,x0,k) / 4;
                d = SrcImg(y1,x1,k) / 4;
                DownSampImg(y,x,k) = a+b+c+d;
            end
        end
    end
end