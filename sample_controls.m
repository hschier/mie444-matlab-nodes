function [left, center, right] = sample_controls(scanMsg)
    cart = readCartesian(scanMsg);
    left = true;
    center = true;
    right = true;
    for i=1:size(cart, 1)
        x = cart(i, 1);
        y = cart(i, 2);
        if (x > 0.1 && x < 0.164 && y < 0.13 && y > -0.13)
            center = false;
        end
        if (x > 0.1 && x < 0.164 && y < 0.12-0.03 && y > -0.12-0.03)
            left = false;
        end
        if (x > 0.1 && x < 0.164 && y < 0.12+0.03 && y > -0.12+0.03)
            right = false;
        end
        
        
    end


end