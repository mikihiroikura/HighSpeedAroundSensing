function fisheye_houghcircle()
%fisheye_houghcircle 魚眼カメラ画像の円のハフ変換による検出
%   詳細説明をここに記述
    %動画からFrameを保存する
    load setup.mat fishbright_dir fish_circle_radi
    vidObj = VideoReader(fishbright_dir);
    img = read(vidObj,1);
    
    %一つ画像を持ってきて，ハフ変換で最も有力の参照面の円を検出
    [centers, radii, metric] = imfindcircles(img,fish_circle_radi);
    
end

