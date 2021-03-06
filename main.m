function main
    % polygon path tracer. two sided.
    
    cam = new_camera(600);
    objects = buildObjects();
    lights =  buildLights();
    
    screen = uint8(zeros( cam.yres, cam.xres, 3));
    
    
   % plot_world(cam, objects); 
   % testPoltIntersect()
   % return;
    
    nSamples = 15;
    frames = 1;
    
    fH = figure(99);
    clf(fH);
    imH = image( screen );
    axis manual
    axis equal
    drawnow
   
    for f = 1:frames,
        fprintf('\nSampling frame %d\n', f);
        moveCam([0,0.4,0]',[0,0,2]');
        buffer = zeros(cam.yres, cam.xres, 3); % RGB buffer
        for s = 1:nSamples,
            parfor row = 1:cam.yres,   
                buffer(row,:,:) =buffer(row,:,:)+ sample(cam, objects, lights, row);     
            end
            
            screen(:,:,1) = uint8(clamp(buffer(:,:,1)./s).*255);
            screen(:,:,2) = uint8(clamp(buffer(:,:,2)./s).*255);
            screen(:,:,3) = uint8(clamp(buffer(:,:,3)./s).*255);
            
            set(imH, 'CData', screen);
            drawnow;
        end
        imwrite(get(imH,'CData'), sprintf('out/frame_%d.png',f), 'png');
    end
    
    set(imH, 'CData', screen);
    drawnow;
           
    fprintf('\nfinished\n');
    
    function moveCam(p,r)
        cam.o = cam.o+p;
        cam.rot = cam.rot + r;
        cam.rot_mat = build_rot_mat(cam.rot(1),cam.rot(2),cam.rot(3));
    end


    function testPoltIntersect()
        r = cam_gen_ray( cam, [0.2,-0.2]);
        p = objects(2);
        %[faceI, t] = faces_intersect(r.o,r.ud,r.l, p.nFaces, p.faceArr); 
        
        %[intersect, isectData ] = intersect_ray_face(r,p.faces(10));
        %isectData.t
        
        o=p.bb.bounds;
        o
        r.sign
        
        intersect = intersect_bb(  r.o, r.sign, r.inv_d, p.bb.bounds );
        intersect
        intersect = intersect_ray_align_box(p.bb,r);
        intersect
    end

end

function buf  = sample(cam, objects, lights,row)
    jitter = cam.pixSize_o2 + (-2*cam.pixSize_o2).*rand(cam.xres,2);
    buf = zeros( 1, cam.xres, 3);
    pCount = 1;
    Y = cam.yres - (row-1);
    for X = 1:cam.xres,
        X_screen = cam.xres - (X-1);
        sx = cam.x_sample(X) + jitter( pCount, 1);
        sy = cam.y_sample(Y) + jitter( pCount, 2);

        c_r = cam_gen_ray( cam, [sx,sy]);
        buf(1,X_screen,:) = radiance(c_r, 0);

        pCount = pCount+1;
    end        
  
        
     function r = radiance(ray, depth)
        intersect = 0;
        isec.t = inf;
        depth = depth+1;
        r = [0,0,0]';
        
        for b = 1:length(objects),  
            [intersectP, isecP]=intersect_ray_polyMEX(ray, objects(b));
            if(intersectP),
                intersect = 1;
                if( isecP.t < isec.t ),
                    isec = isecP;
                end
            end
        end

        if(~intersect || depth>=4) % no intersect or end of depth
            return;
        end

        %move ip out a bit to avoid self intersect
        isec.ip = isec.ip + 0.0001*isec.n;
        
        % check shadow rays and add radiance
        for l = 1:length(lights)
           d = lights(l).p - isec.ip; 
           dl = l_v(d);
           ray_shadow = make_ray( isec.ip, d, dl); 
           
           for b = 1:length(objects),  
              intersect=shadow_ray_polyMEX(ray_shadow, objects(b));
              if(intersect), break; end  
           end 
           if(~intersect),
             % fprintf('no shadow\n');
             flux = sum(ray_shadow.ud.*isec.n) ; % dot prod
             r = r + isec.c.*( (lights(l).e/sqrt(dl))*flux);
           end
        end
        
        % reflection ray
        %ref_d = ray.d - 2*(ray.d'*isec.n)*isec.n;
        %rayRef = make_ray(isec.ip, ref_d, 100);
              
        % random direction    
        rd = randn(3,1);
        if(sum(rd.*isec.n) < 0), rd = -rd; end% dot product, flip if behind norm
        rayRef = make_ray(isec.ip, rd, 100);
        
        % send out GI ray
        %flux = sum(conj(rayRef.ud).*isec.n) ; % dot prod
        %r = r + isec.c.*(radiance(rayRef, depth)*flux);
        r = r + isec.c.*radiance(rayRef, depth);
     end

end
    
function objects = buildObjects
    objects(1) = make_plane(25,14, [1,1,1]', [0,0,0]');
    objects(1) = poly_translate(objects(1), [15.5,10.5,0.15]');
 
    objects(2) = make_box(2,4,1, [0.75,0.25,0.25]', [0,0,0]');
    objects(2) = poly_translate(objects(2), [10,10,1.5]');
    rot_mat = build_rot_mat(2,2,2);
    objects(2) = poly_rotate(objects(2), rot_mat);
    
    objects(3) = make_box(2,3,3, [0.25,0.25,0.75]', [0,0,0]');
    objects(3) = poly_translate(objects(3), [13,9,3]');
    rot_mat = build_rot_mat(5,25,5);
    objects(3) = poly_rotate(objects(3), rot_mat);
    
   % objects(4) = make_plane(25,14, [1,1,1]', [0,0,0]');
   % objects(4) = poly_translate(objects(4), [15.5,10.5,11.15]');
    
 %   objects(3) = make_box(14,14,0.3, [0.25,0.75,0.25]', [0,0,0]');
 %   objects(3) = poly_translate(objects(3), [10.5,10.5,0.15]');
 %   objects(3).faces = flipNormals(objects(3).faces);

    
    
end

function lights =  buildLights()
    lights(1).e = [3,3,3]'; % light emision
    lights(1).p = [0,5,10]';
end

function a = clamp(a)
    a(a>1) =1;
    a(a<0) =0;
end

function camera = new_camera(res)
    % camera space has camera at [0 0 0], and pointing down x axis
    % make a ray and rotate and translate to world space
    camera.fov = 45;
    camera.o = [3,2,3]'; % camera location
    rot = [0,-6,-40]';  %x - roll, y- pitch, z-yaw
    camera.rot = rot; % rotation from origin
    camera.yres = res;
    camera.aspect = 1;
    camera.sample = 1; % current sample
    
    camera.xres = floor( camera.yres * camera.aspect );
    camera.screenSize = camera.yres * camera.xres;
    camera.yon = 1; % distance to image plane
    
    camera.rot_mat = build_rot_mat(rot(1),rot(2),rot(3));
    
    % image maps to -1,1 in z
    camera.sz = camera.yon*tand(camera.fov/2); % half screen height (z)
    
    camera.pixSize = (2/camera.yres);
    camera.pixSize_o2 = camera.pixSize/2;
    camera.y_sample = -1:camera.pixSize:2;
    camera.x_sample = (-1*camera.aspect):camera.pixSize:(camera.aspect+1);
    % force correct size
    camera.y_sample = camera.y_sample(1:camera.yres);
    camera.x_sample = camera.x_sample(1:camera.xres);

end

function r = cam_gen_ray( cam, imagecoord)
    % generate a ray through cam
    %imagecoord [y,z]
    screen_p = [1,  (imagecoord*cam.sz)  ]';
    
    screen_p_len = l_v(screen_p);
    screen_p = screen_p./screen_p_len; % unit vec
    screen_p = (screen_p'*cam.rot_mat)'; % rotates around zero
    
    r = make_ray( cam.o, screen_p,100);
end

function r = make_ray(o, d, l)
    % origin, direction
    
    r.o = o;
    r.l = l; % ray length
    
    ld=l_v(d); 
    r.ud = d/ld; % unit direction
    
    r.d = r.ud.*r.l; % full length vector
    r.inv_d = 1./d;
    r.sign = double(r.inv_d < 0);

end

function bb = make_bounding_box(p)
  % take poly and make a bounding box
  n = length(p.faces); % n faces
  x_co = zeros( n,3);
  y_co = zeros( n,3);
  z_co = zeros( n,3);
  
  for i = 1:n
      x_co(i,1) = p.faces(i).v1(1);
      x_co(i,2) = p.faces(i).v2(1);
      x_co(i,3) = p.faces(i).v3(1);
      
      y_co(i,1) = p.faces(i).v1(2);
      y_co(i,2) = p.faces(i).v2(2);
      y_co(i,3) = p.faces(i).v3(2);
      
      z_co(i,1) = p.faces(i).v1(3);
      z_co(i,2) = p.faces(i).v2(3);
      z_co(i,3) = p.faces(i).v3(3);
      
  end  
  
  minP = [ min(x_co(:)), min(y_co(:)),min(z_co(:))]';
  maxP = [ max(x_co(:)), max(y_co(:)),max(z_co(:))]';
  bb = make_align_box(maxP, minP);
end

function b = make_align_box(maxP, minP)
% box is defined by two co-ords and is axis aligned
% order them in space
    b.bounds = zeros(3,2);
    b.bounds(:,1) = min( [minP,maxP], [], 2); %x,y,z min
    b.bounds(:,2) = max( [minP,maxP], [], 2);  %x,y,z max  
end   

function intersect = intersect_ray_align_box(b,r)

    tmin = ( b.bounds(1,r.sign(1)+1) - r.o(1))*r.inv_d(1);
    tmax = ( b.bounds(1,2-r.sign(1)) - r.o(1))*r.inv_d(1);
    tymin = ( b.bounds(2,r.sign(2)+1) - r.o(2))*r.inv_d(2);
    tymax = ( b.bounds(2,2-r.sign(2)) - r.o(2))*r.inv_d(2);
    
 %  tmin = (bounds[r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
 %  tmax = (bounds[1-r.sign[0]].x() - r.origin.x()) * r.inv_direction.x(); 
 %  tymin = (bounds[r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
 %  tymax = (bounds[1-r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
 
     if ( (tmin > tymax) || (tymin > tmax) ),
         intersect = 0;
         return;
     end
     if (tymin > tmin)
           tmin = tymin;
     end
     
     if (tymax < tmax)
          tmax = tymax;
     end
     
     tzmin = ( b.bounds(3,r.sign(3)+1) - r.o(3))*r.inv_d(3);
     tzmax = ( b.bounds(3,2-r.sign(3)) - r.o(3))*r.inv_d(3);
    
 %   tzmin = (bounds[r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
 %   tzmax = (bounds[1-r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
 
     if ( (tmin > tzmax) || (tzmin > tmax) )
        intersect = 0;
        return;
     end

    intersect = 1;

end

function [intersect, isectData ]= intersect_ray_face(r,f)
    % M�ller-Trumbore algorithm
    % isectData - pint of intersect
    % intersect . intersects?
    % t, distance from ray origin of intersect
    % u,v are Barycentric Coordinates
     intersect = 0;
    
     edge1 = f.v2 - f.v1;
     edge2 = f.v3 - f.v1;
     pvec = mcross(r.ud, edge2);
     %det = dot(edge1, pvec);
     det = sum(conj(edge1).*pvec);
     
     if (det == 0)
         isectData=0;
         return;
     end
     invDet = 1 / det;
     tvec = r.o - f.v1;
     %isectData.u = dot(tvec, pvec) * invDet;
     isectData.u = sum(conj(tvec).*pvec) * invDet;
     if (isectData.u < 0 || isectData.u > 1)
         return;
     end
     qvec = mcross(tvec, edge1);
     %isectData.v = dot(r.ud, qvec) * invDet;
     isectData.v = sum(conj(r.ud).*qvec) * invDet;
     if (isectData.v < 0 || isectData.u + isectData.v > 1)
         return;
     end
     %isectData.t = dot(edge2, qvec) * invDet;
     isectData.t = sum(conj(edge2).*qvec)  * invDet;
     
     if((isectData.t < 0.001) ||  (isectData.t>r.l)), return; end
    
     isectData.n = f.n;
     intersect = 1;
end

function c=mcross(a,b)
c = [a(2).*b(3)-a(3).*b(2); 
        a(3).*b(1)-a(1).*b(3); 
        a(1).*b(2)-a(2).*b(1)];
end

function [intersect, isecData] = intersect_ray_poly(r, p)
    isecData.t= inf;
    
    % check bounding box
   intersect = intersect_ray_align_box(p.bb,r);
   if(~intersect), return; end
    
   intersect = 0;
   
   for i = 1:length(p.faces) % for each face
        [inter, data ] = intersect_ray_face(r,p.faces(i));
        if(inter),
            intersect = 1;
            if( data.t < isecData.t),
                isecData = data;
            end
        end
   end
   
   % intersect coord
   if(intersect)
    isecData.ip = r.o + r.ud*isecData.t;
    isecData.c = p.c; % material. colour
    isecData.e = p.e; % material emision
   end
end

function [intersect, isecData] = intersect_ray_polyMEX(r, p)
    isecData.t = inf;
    
    % check bounding box
   %intersect = intersect_bb(  r.o, r.sign, r.inv_d, p.bb.bounds );
   intersect = intersect_ray_align_box(p.bb,r);
   if(~intersect), return; end
   
   intersect = 0;
   [faceI, t] = faces_intersect(r.o,r.ud,r.l, p.nFaces, p.faceArr); 
    
    % intersect coord
   if(faceI)
    intersect = 1;
    isecData.t = t;
    isecData.n = p.faces(faceI).n;
    isecData.ip = r.o + r.ud*t;
    isecData.c = p.c; % material. colour
    isecData.e = p.e; % material emision
   end
    
end

function intersect = shadow_ray_polyMEX(r, p)
    % check bounding box
  % intersect = intersect_bb(  r.o, r.sign, r.inv_d, p.bb.bounds );
   intersect = intersect_ray_align_box(p.bb,r);
   if(~intersect), return; end   
   intersect = faces_intersect_shadow(r.o,r.ud,r.l, p.nFaces, p.faceArr); 
end

function plot_bounding_box(p)
    b = p.bb;
    lX = zeros( 2, 12);
    lY = zeros( 2, 12);
    lZ = zeros( 2, 12);
    
    lX( :,1:4) = b.bounds(1,1);
    lX( :,5:8) = b.bounds(1,2);
    lX( 1,9:12) = b.bounds(1,1);
    lX( 2,9:12) = b.bounds(1,2);
    
    lY( :, [1,10,9,5]) = b.bounds(2,1);
    lY( :, [3,11,7,12]) = b.bounds(2,2);
    lY( 1, [4,2,6,8]) = b.bounds(2,1);
    lY( 2, [4,2,6,8]) = b.bounds(2,2);
    
    lZ( :, [9,4,12,8]) = b.bounds(3,1);
    lZ( :, [10,2,11,6]) = b.bounds(3,2);
    lZ( 1, [1,3,7,5]) = b.bounds(3,1);
    lZ( 2, [1,3,7,5]) = b.bounds(3,2);
    
    plot3( lX,lY,lZ, 'k--');
    
end

function plot_point(p)
    plot3( p(1), p(2), p(3), 'x');
end

function plot_ray(r)
    e = r.o + (r.d);  
    plot3( [r.o(1); e(1)],[r.o(2); e(2)],[r.o(3);e(3)]); 
end

function box = make_box(w,d,h,c,e)

    % make a box from triangles
    faces(12).v1 = zeros( 3,1); % verts of triangle. x,y,z
    faces(12).v2 = zeros( 3,1); % verts of triangle. x,y,z
    faces(12).v3 = zeros( 3,1); % verts of triangle. x,y,z
    faces(12).n = zeros( 3,1); % normal
    
    verts = zeros( 3,8);
    verts(:,1) = [0,0,0]; verts(:,2) = [0,0,h]; 
    verts(:,3) = [w,0,0]; verts(:,4) = [w,0,h];
    verts(:,5) = [w,d,0]; verts(:,6) = [w,d,h]; 
    verts(:,7) = [0,d,0]; verts(:,8) = [0,d,h];
       
    % anti-clockwise!
    faces(1).v1 = verts(:,1); faces(1).v2 = verts(:,3); faces(1).v3 = verts(:,2);
    faces(2).v1 = verts(:,2); faces(2).v2 = verts(:,3); faces(2).v3 = verts(:,4);
    
    faces(3).v1 = verts(:,3); faces(3).v2 = verts(:,5); faces(3).v3 = verts(:,4);
    faces(4).v1 = verts(:,4); faces(4).v2 = verts(:,5); faces(4).v3 = verts(:,6);
    
    faces(5).v1 = verts(:,5); faces(5).v2 = verts(:,7); faces(5).v3 = verts(:,6);
    faces(6).v1 = verts(:,6); faces(6).v2 = verts(:,7); faces(6).v3 = verts(:,8);
    
    faces(7).v1 = verts(:,7); faces(7).v2 = verts(:,1); faces(7).v3 = verts(:,8);
    faces(8).v1 = verts(:,8); faces(8).v2 = verts(:,1); faces(8).v3 = verts(:,2);
    
    faces(9).v1 = verts(:,1); faces(9).v2 = verts(:,7); faces(9).v3 = verts(:,3);
    faces(10).v1 = verts(:,3); faces(10).v2 = verts(:,7); faces(10).v3 = verts(:,5);
    
    faces(11).v1 = verts(:,2); faces(11).v2 = verts(:,4); faces(11).v3 = verts(:,8);
    faces(12).v1 = verts(:,4); faces(12).v2 = verts(:,6); faces(12).v3 = verts(:,8);
    
    box.faces = faces;
    box.nFaces = 12;
    box = update_normals(box); % normals
    box.faceArr = make_faceArr(faces);
    
    box.bb = make_bounding_box(box); % bounding box
    box.o = [0,0,0]'; % origin
    
    % centre origin
    box = poly_translate(box, [-w/2, -d/2, -h/2]');
    box.o = [0,0,0]'; % reset origin
    box.c = c; % colour
    box.e = e; % emmission
    
end

function plane = make_plane(w,h,c,e)
    % make a plane from triangles
    faces(2).v1 = zeros( 3,1); % verts of triangle. x,y,z
    faces(2).v2 = zeros( 3,1); % verts of triangle. x,y,z
    faces(2).v3 = zeros( 3,1); % verts of triangle. x,y,z
    faces(2).n = zeros( 3,1); % normal
    
    
    faces(1).v1 = [0,0,0]'; faces(1).v2 = [w,0,0]'; faces(1).v3 = [0,h,0]';
    faces(2).v1 = [w,0,0]'; faces(2).v2 = [w,h,0]'; faces(2).v3 = [0,h,0]';
    
    plane.faces = faces;
    plane.nFaces = 2;
    plane = update_normals(plane); % normals
    plane.faceArr = make_faceArr(faces);
    
    plane.bb = make_bounding_box(plane); % bounding box
    plane.o = [0,0,0]'; % origin
    
    % centre origin
    plane = poly_translate(plane, [-w/2, -h/2, 0]');
    plane.o = [0,0,0]'; % reset origin
    plane.c = c; % colour
    plane.e = e; % emmission

end

function fa = make_faceArr(faces)
% 1d array of verts 
    fn = size(faces,2);
    fa = zeros( fn*9, 1 );

    for f = 1:fn,
       idx = ((f-1)*9) + (1:9);     
       fa(idx) = [ faces(f).v1,faces(f).v2,faces(f).v3 ];        
    end
end

function plot_poly(b)

    for i = 1:length(b.faces) % for each face
        
        lX = build_tri_coords(b.faces(i),1);   
        lY = build_tri_coords(b.faces(i),2); 
        lZ = build_tri_coords(b.faces(i),3); 
        
        plot3( lX,lY,lZ, 'Color', clamp(b.c));
    end

end

function plot_poly_norms(b, s)
% s size
    c = zeros(3,1); % centre
    for i = 1:length(b.faces) % for each face
        f = b.faces(i);
        
        c(1) = sum( [f.v1(1), f.v2(1), f.v3(1)] )/3;
        c(2) = sum( [f.v1(2), f.v2(2), f.v3(2)] )/3;
        c(3) = sum( [f.v1(3), f.v2(3), f.v3(3)] )/3;
        n = c + f.n*s;  
        plot3( [c(1) ; n(1)],[c(2); n(2)],[c(3);n(3)], 'k'); 

    end
end

function plot_world(cam, objects)
     
    fh = figure(1);
    clf(fh);
    set(gca, 'Projection', 'perspective');
    set(gca, 'CameraViewAngleMode', 'manual');
    axis manual
    axis ([0, 1,0, 1,0, 1 ].*15);
    hold on
      
    for i = 0:0
        cla(gca); 
        % camera ray sweep
        %v=floor(i/cam.xres);
        %u = i-(v*cam.xres);
        %v = ((v/cam.yres)*2)-1; % between -1 and 1
        %u = ((u/cam.xres)*2)-1;
        %fprintf('u: %.2f, v: %.2f\n', u,v );
        c_r = cam_gen_ray( cam, [0.2,-0.2]);
        plot_ray(c_r);
        
        
        for b = 1:length(objects),
            % box = poly_rotate(box, rot_mat);
            plot_poly(objects(b));
            plot_poly_norms(objects(b), 2);
            plot_bounding_box(objects(b));
            
            [intersect, isecData]=intersect_ray_poly(c_r, objects(b));
            if(intersect),
                plot_point( isecData.ip);
                
                %reflection ray
                ref_d = c_r.d - 2*(c_r.d'*isecData.n)*isecData.n;
                
                ref_ray = make_ray( isecData.ip, ref_d, 100);
                plot_ray(ref_ray);
            end
        end

        drawnow
    end

end

function lX = build_tri_coords(face,a)
        % face i, axis a
        lX = zeros( 2, 3);
        
        lX(1,1) = face.v1(a);
        lX(2,1) = face.v2(a);
        lX(1,2) = face.v2(a);
        lX(2,2) = face.v3(a);
        lX(1,3) = face.v3(a);
        lX(2,3) = face.v1(a);
end

function b = poly_translate(b, t)

    for i = 1:length(b.faces) 
        b.faces(i).v1 = b.faces(i).v1 + t;
        b.faces(i).v2 = b.faces(i).v2 + t;
        b.faces(i).v3 = b.faces(i).v3 + t;    
    end
    b.o = b.o+ t;
    b.bb.bounds(:,1) = b.bb.bounds(:,1) + t;
    b.bb.bounds(:,2) = b.bb.bounds(:,2) + t;
    b.faceArr = make_faceArr(b.faces);    
end

function b = poly_rotate(b, rot_mat)
 
    for i = 1:length(b.faces)
       
        b.faces(i).v1 = ((b.faces(i).v1 - b.o)' * rot_mat)'+b.o;
        b.faces(i).v2 = ((b.faces(i).v2 - b.o)' * rot_mat)'+b.o;
        b.faces(i).v3 = ((b.faces(i).v3 - b.o)' * rot_mat)'+b.o;
        
    end
    b = update_normals(b); % normals
    b.bb = make_bounding_box(b); % bounding box
    b.faceArr = make_faceArr(b.faces);   
end

function rot_mat = build_rot_mat(xa,ya,za) % rot in degs

    rot_matX = [1 0 0 
                0 cosd(xa) -sind(xa) 
                0 sind(xa) cosd(xa) ]; 
    rot_matY = [cosd(ya) 0 sind(ya) 
                0 1 0 
                -sind(ya) 0 cosd(ya) ]; 
    rot_matZ = [cosd(za) -sind(za) 0 
                sind(za) cosd(ya) 0 
                0 0 1 ]; 
    rot_mat = rot_matX*rot_matY*rot_matZ;

end

function ob = update_normals(ob)
    
    for i = 1:length(ob.faces)
        ob.faces(i).n = compute_normal(ob.faces(i));
    end
    
    fn = size(ob.faces,2);
    nArr = zeros( 3, fn );

    for f = 1:fn,     
       nArr(:,f) = ob.faces(f).n ;        
    end
    
    ob.nArr = nArr;
end

function n = compute_normal(f)
    % compute normal from face
    U = f.v2 - f.v1;
    V = f.v3 - f.v1;
    n = cross(f.v2 - f.v1, f.v3-f.v1);
    n = n./l_v(n); % normalise
end

% function faces = flipNormals(faces)
% 
%   % rotation will flip them back
%    for i = 1:length(faces)
%         faces(i).n = faces(i).n.*-1;
%    end
% end

function l=l_v(v)
    % vector length
    l = sqrt( sum( v.^2));
end

