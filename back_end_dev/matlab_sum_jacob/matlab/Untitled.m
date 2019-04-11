            dvo.moved_image= (zeros(dvo.v_max,dvo.u_max));
            dvo.fixed_image2= (zeros(dvo.v_max,dvo.u_max));
            for i = 1:size(dvo.U)
                    dvo.moved_image(dvo.V(i),dvo.U(i))= 3;
%                     dvo.fixed_image2(dvo.V_fixed(i),dvo.U_fixed(i))= intensity_x1(i);
%                     dvo.fixed_image2(dvo.V(i),dvo.U(i))= dvo.fixed_image(dvo.V(i),dvo.U(i));
                    dvo.fixed_image2(dvo.V(i),dvo.U(i))= 2;
            end
            for i = 1:size(dvo.U_fixed,1)
%                     dvo.fixed_image2(dvo.V_fixed(i),dvo.U_fixed(i))= dvo.fixed_image(dvo.V_fixed(i), dvo.U_fixed(i));
            end
            figure(2);
            imshow(dvo.moved_image);
            title('Moved Image');
            % hold on;
            figure(3);
            imshow(dvo.fixed_image2);
            title('Fixed Image'); 
            figure(4);
            imshow(dvo.fixed_image);
            title('ref');