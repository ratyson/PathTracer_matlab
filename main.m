function main
    % polygon path tracer. two sided.
    
    cam = new_camera();
    objects = buildObjects();
    
    plot_world(cam, objects); 
 
    nSamples = 2500;
    
    fH = figure(99);
    clf(fH);
    imH = image(cam.screen);
    axis manual
    axis equal
    drawnow
   
    for s = 1:nSamples, 
       sample(s);
       set(imH, 'CData', cam.screen);
       drawnow
       s
    end
    

  %  b = make_align_box( [1,20,4]' , [6,5,2]');   
   % d = [1,1,0.8]';
   % r = make_ray([0,0,0]', d); 
  %  intersect_ray_align_box(b,c_r);

  function sample(n)
    jitter = cam.pixSize_o2 + (-2*cam.pixSize_o2).*rand(cam.screenSize,2);
   
    pCount = 1;
    for Y = 1:cam.yres,  
        for X = 1:cam.xres,
            sx = cam.x_sample(X) + jitter( pCount, 1);
            sy = cam.y_sample(Y) + jitter( pCount, 2);
            
            c_r = cam_gen_ray( cam, [sx,sy]);
            
            col = radiance(c_r, 1);
               
            cam.buffer(Y,X,1) = cam.buffer(Y,X,1) + col(1);
            cam.buffer(Y,X,2) = cam.buffer(Y,X,2) + col(2);
            cam.buffer(Y,X,3) = cam.buffer(Y,X,3) + col(3);
            cam.screen(Y,X,:) = uint8(clamp(cam.buffer(Y,X,:)./n)*255); 
            
        
            pCount = pCount+1;
        end        
    end
    
  end

    function r = radiance(ray, depth)
        intersect = 0;
        isec.t = inf;
        depth = depth+1;
        for b = 1:length(objects),  
            [intersectP, isecP]=intersect_ray_poly(ray, objects(b));
            if(intersectP),
                intersect = 1;
                if( isecP.t < isec.t ),
                    isec = isecP;
                end
            end
        end

        if(~intersect) 
            r = [0,0,0];
            return;
        end

        if( depth>=5)
            r = isec.e;
            return;
        end
        rn = (rand(3,1).*180)-90;
        rotMat = build_rot_mat(rn(1),rn(2),rn(3));
        d = isec.n'*rotMat;
        rayRef = make_ray(isec.ip, d');
        r = isec.e + isec.c.*(radiance(rayRef, depth)*(1/isec.t));
    
    end

end





function objects = buildObjects

    objects(1) = make_box(2,3,3, [0.25,0.25,0.75], [0,0,0]);
    objects(1) = poly_translate(objects(1), [13,9,3]');
    rot_mat = build_rot_mat(5,25,5);
    objects(1) = poly_rotate(objects(1), rot_mat);
    
    
    objects(2) = make_box(2,4,1, [0.75,0.25,0.25], [0,0,0]);
    objects(2) = poly_translate(objects(2), [10,10,3]');
    rot_mat = build_rot_mat(2,2,2);
    objects(2) = poly_rotate(objects(2), rot_mat);
    
    
    objects(3) = make_box(5,5,5, [0,0,0], [8,8,8]); % light
    objects(3) = poly_translate(objects(3), [3,3,12]');
    
   % objects(4) = make_box(14,14,14, [0.25,0.75,0.25], [0,0,0]);
   % objects(4) = poly_translate(objects(4), [7.5,7.5,7.5]');
   % objects(4).faces = flipNormals(objects(4).faces);
    
end

function a = clamp(a)
    a(a>1) =1;
    a(a<0) =0;
end

function camera = new_camera()
    % camera space has camera at [0 0 0], and pointing down x axis
    % make a ray and rotate and translate to world space
    camera.fov = 45;
    camera.o = [3,3,3]'; % camera location
    rot = [0,0,-45]';  %x - roll, y- pitch, z-yaw
    camera.rot = rot; % rotation from origin
    camera.yres = 50;
    camera.aspect = 1;
    camera.sample = 1; % current sample
    
    camera.xres = floor( camera.yres * camera.aspect );
    camera.screenSize = camera.yres * camera.xres;
    camera.screen = uint8(zeros( camera.yres, camera.xres, 3));
    camera.buffer = zeros( camera.yres, camera.xres, 3);
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
    
    size(camera.x_sample)
    size(camera.y_sample)
end

function r = cam_gen_ray( cam, imagecoord)
    % generate a ray through cam
    %imagecoord [y,z]
    screen_p = [1,  (imagecoord*cam.sz)  ]';
    
    screen_p_len = l_v(screen_p);
    screen_p = screen_p./screen_p_len; % unit vec
    screen_p = (screen_p'*cam.rot_mat)'; % rotates around zero
    
    r = make_ray( cam.o, screen_p);
end

function r = make_ray(o, d)
    % origin, direction
    
    r.o = o;
    r.l = 50; % ray length
    
    ld=l_v(d); 
    r.ud = d/ld; % unit direction
    
    r.d = r.ud.*r.l; % full length vector
    r.inv_d = 1./d;
    r.sign = r.inv_d < 0;

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
     pvec = cross(r.ud, edge2);
     det = dot(edge1, pvec);
     if (det == 0)
         isectData=0;
         return;
     end
     invDet = 1 / det;
     tvec = r.o - f.v1;
     isectData.u = dot(tvec, pvec) * invDet;
     if (isectData.u < 0 || isectData.u > 1)
         return;
     end
     qvec = cross(tvec, edge1);
     isectData.v = dot(r.ud, qvec) * invDet;
     if (isectData.v < 0 || isectData.u + isectData.v > 1)
         return;
     end
     isectData.t = dot(edge2, qvec) * invDet;
     
     if(isectData.t < 0.001), return; end
    
     isectData.n = f.n;
     intersect = 1;
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
    
    faces = update_normals(faces); % normals
    box.faces = faces;
    
    
    box.bb = make_bounding_box(box); % bounding box
    box.o = [0,0,0]'; % origin
    
    % centre origin
    box = poly_translate(box, [-w/2, -d/2, -h/2]');
    box.o = [0,0,0]'; % reset origin
    box.c = c; % colour
    box.e = e; % emmission
    
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
        c_r = cam_gen_ray( cam, [0,0]);
        plot_ray(c_r);
        
        
        for b = 1:length(objects),
            % box = poly_rotate(box, rot_mat);
            plot_poly(objects(b));
            plot_poly_norms(objects(b), 2);
            plot_bounding_box(objects(b));
            
            [intersect, isecData]=intersect_ray_poly(c_r, objects(b));
            if(intersect),
                plot_point( isecData.ip);
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
        
end

function b = poly_rotate(b, rot_mat)
 
    for i = 1:length(b.faces)
       
        b.faces(i).v1 = ((b.faces(i).v1 - b.o)' * rot_mat)'+b.o;
        b.faces(i).v2 = ((b.faces(i).v2 - b.o)' * rot_mat)'+b.o;
        b.faces(i).v3 = ((b.faces(i).v3 - b.o)' * rot_mat)'+b.o;
        
    end
    b.faces = update_normals(b.faces); % normals
    b.bb = make_bounding_box(b); % bounding box
        
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

function faces = update_normals(faces)
    
    for i = 1:length(faces)
        faces(i).n = compute_normal(faces(i));
    end

end

function n = compute_normal(f)
    % compute normal from face
    U = f.v2 - f.v1;
    V = f.v3 - f.v1;
    n = cross(f.v2 - f.v1, f.v3-f.v1);
    n = n./l_v(n); % normalise
end

function faces = flipNormals(faces)

  % rotation will flip them back
   for i = 1:length(faces)
        faces(i).n = faces(i).n.*-1;
   end
end

function l=l_v(v)
    % vector length
    l = sqrt( sum( v.^2));
end

