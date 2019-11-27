function forward = can_move_foward(scanMsg)
    cart = readCartesian(scanMsg);
    forward = true;
    for i=1:size(cart, 1)
        x = cart(i, 1);
        y = cart(i, 2);
        if (x > 0.1 && x < 0.15 && y < 0.115 && y > -0.115)
            forward = false;
            break
        end
    end

end