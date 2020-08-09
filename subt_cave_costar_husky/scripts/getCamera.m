function img = getCamera(camera_msg)

    height = camera_msg.Height;
    width = camera_msg.Width;
    img_raw_R = reshape(camera_msg.Data(1:3:end),[width, height])';
    img_raw_G = reshape(camera_msg.Data(2:3:end),[width, height])';
    img_raw_B = reshape(camera_msg.Data(3:3:end),[width, height])';
    img_raw(:,:,1) = img_raw_R;
    img_raw(:,:,2) = img_raw_G;
    img_raw(:,:,3) = img_raw_B;
    img = uint8(img_raw);