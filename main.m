function main
    % polygon path tracer. two sided.
    
    cam = new_camera();
    c_r = cam_gen_ray( cam, [0,0]);
    
    b = make_align_box( [1,20,4]' , [6,5,2]');
    
    d = [1,1,0.8]';
   % r = make_ray([0,0,0]', d);
    
    box = make_box(2,3,3, [0,0,1]);
    box = poly_translate(box, [10,10,3]');
    rot_mat = build_rot_mat(2,2,2);
    box = poly_rotate(box, rot_mat);
    
    intersect_ray_align_box(b,c_r);
     
    fh = figure(1);
    clf(fh);
    set(gca, 'Projection', 'perspective');
    set(gca, 'CameraViewAngleMode', 'manual');
    axis manual
    axis ([0, 1,0, 1,0, 1 ].*15);
    hold on
    
    plot_poly(box);
    plot_poly_norms(box, 2);
    plot_ray(c_r);
    
    for i = 1:600
        cla(gca);    
        box = poly_rotate(box, rot_mat);
        plot_poly(box);
        %plot_poly_norms(box, 2);
        %plot_bounding_box(box);
        plot_ray(c_r);
        [intersect, isecData]=intersect_ray_poly(c_r, box);
        if(intersect),
            plot_point( isecData.ip);
        end
        drawnow
    end

end

function camera = new_camera()
    % camera space has camera at [0 0 0], and pointing down x axis
    % make a ray and rotate and translate to world space
    camera.fov = 45;
    camera.o = [1,1,1]'; % camera location
    rot = [45,5,-45]';
    camera.rot = rot; % rotation from origin
    camera.yres = 320;
    camera.aspect = 16/9;
    camera.sample = 1; % current sample
    
    camera.xres = floor( camera.yres * camera.aspect );
    camera.screen = zeros( camera.yres, camera.xres);
    camera.buffer = zeros(camera.yres, camera.xres);
    camera.yon = 1; % distance to image plane
    
    camera.rot_mat = build_rot_mat(rot(1),rot(2),rot(3));
    
    % image maps to -1,1 in z
    camera.sz = camera.yon*tand(camera.fov/2); % half screen height (z)
    
end

function r = cam_gen_ray( cam, imagecoord)
    % generate a ray through cam
    %imagecoord [y,z]
    screen_p = [1,  (imagecoord*cam.sz)  ]'
    r = make_ray( [0,0,0]', screen_p);
    
    r.d = (r.d'*cam.rot_mat)'; % rotates around zero
    r.o = r.o + cam.o;
    %rotate to camera direction and move to cam origin    
end

function r = make_ray(o, d)
    % origin, direction
    
    r.o = o;
    ld=l_v(d);
    r.d = d/ld;
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
         fprintf('b\n');
        intersect = 0;
        return;
     end

    intersect = 1;

end

function [intersect, isectData ]= intersect_ray_face(r,f)
    % Möller-Trumbore algorithm
    % isectData - pint of intersect
    % intersect . intersects?
    % t, distance from ray origin of intersect
    % u,v are Barycentric Coordinates
     intersect = 0;
    
     edge1 = f.v2 - f.v1;
     edge2 = f.v3 - f.v1;
     pvec = cross(r.d, edge2);
     det = dot(edge1, pvec);
     if (det == 0)
         return;
     end
     invDet = 1 / det;
     tvec = r.o - f.v1;
     isectData.u = dot(tvec, pvec) * invDet;
     if (isectData.u < 0 || isectData.u > 1)
         return;
     end
     qvec = cross(tvec, edge1);
     isectData.v = dot(r.d, qvec) * invDet;
     if (isectData.v < 0 || isectData.u + isectData.v > 1)
         return;
     end
     isectData.t = dot(edge2, qvec) * invDet;
     
     intersect = 1;
end

function [intersect, isecData] = intersect_ray_poly(r, p)
    intersect = 0;
    isecData.t= inf;
    
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
   isecData.ip = r.o + r.d*isecData.t;

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
    e = r.o + (r.d*50);  
    plot3( [r.o(1); e(1)],[r.o(2); e(2)],[r.o(3);e(3)]); 
end

function box = make_box(w,d,h,c)

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
    
    
end

function plot_poly(b)

    for i = 1:length(b.faces) % for each face
        
        lX = build_tri_coords(b.faces(i),1);   
        lY = build_tri_coords(b.faces(i),2); 
        lZ = build_tri_coords(b.faces(i),3); 
        
        plot3( lX,lY,lZ, 'Color', b.c);
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

function l=l_v(v)
    % vector length
    l = sqrt( sum( v.^2));
end

