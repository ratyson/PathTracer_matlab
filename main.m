function main
    % box path tracer. two sided. 
    b = make_align_box( [1,20,4]' , [6,5,2]');
    
    d = [1,1,0.8]';
    l_d = sqrt( sum( d.^2));
    d = d./l_d;
    r = make_ray([0,0,0]', d);
    
    box = make_box(4,5,6, [0,0,1]);
    box = poly_translate(box, [1,1,1]');
    
    rot_mat = build_rot_mat(2,2,2);
    box = poly_rotate(box, rot_mat);
    
    fh = figure(1);
    clf(fh);
    set(gca, 'Projection', 'perspective');
    set(gca, 'CameraViewAngleMode', 'manual');
    axis manual
    axis ([-20, 20,-20, 20,-20, 20 ]);
    hold on
    
    for i = 1:600
        cla(gca);
        plot_box(box);
        box = poly_rotate(box, rot_mat);
        drawnow
    end
    %plot_align_box(b);
    plot_box(box);
    intersect_ray_align_box(b,r);

end

function r = make_ray(o, d)
    % origin, direction
    
    r.o = o;
    r.d = d;
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
         fprintf('a\n');
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

function plot_align_box(b)
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
       
    % clockwise!
    faces(1).v1 = verts(:,1); faces(1).v2 = verts(:,2); faces(1).v3 = verts(:,3);
    faces(2).v1 = verts(:,2); faces(2).v2 = verts(:,4); faces(2).v3 = verts(:,3);
    
    faces(3).v1 = verts(:,3); faces(3).v2 = verts(:,4); faces(3).v3 = verts(:,5);
    faces(4).v1 = verts(:,4); faces(4).v2 = verts(:,6); faces(4).v3 = verts(:,5);
    
    faces(5).v1 = verts(:,5); faces(5).v2 = verts(:,6); faces(5).v3 = verts(:,7);
    faces(6).v1 = verts(:,6); faces(6).v2 = verts(:,8); faces(6).v3 = verts(:,7);
    
    faces(7).v1 = verts(:,7); faces(7).v2 = verts(:,8); faces(7).v3 = verts(:,1);
    faces(8).v1 = verts(:,8); faces(8).v2 = verts(:,2); faces(8).v3 = verts(:,1);
    
    faces(9).v1 = verts(:,1); faces(9).v2 = verts(:,3); faces(9).v3 = verts(:,7);
    faces(10).v1 = verts(:,3); faces(10).v2 = verts(:,5); faces(10).v3 = verts(:,7);
    
    faces(11).v1 = verts(:,2); faces(11).v2 = verts(:,8); faces(11).v3 = verts(:,4);
    faces(12).v1 = verts(:,4); faces(12).v2 = verts(:,8); faces(12).v3 = verts(:,6);
    
    box.faces = faces;
    box.bb = make_bounding_box(box); % bounding box
    box.o = [0,0,0]'; % origin
    
    % centre origin
    box = poly_translate(box, [-w/2, -d/2, -h/2]');
    box.o = [0,0,0]'; % reset origin
    box.c = c; % colour
    
    
end

function plot_box(b)

    for i = 1:12 % for each face
        
        lX = build_tri_coords(b.faces(i),1);   
        lY = build_tri_coords(b.faces(i),2); 
        lZ = build_tri_coords(b.faces(i),3); 
        
        plot3( lX,lY,lZ, 'Color', b.c);
    end

    plot_align_box(b.bb);
end

function lX = build_tri_coords(face,a)
        % face i axis a
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
    
    b.bb.bounds(:,1) = b.bb.bounds(:,1) + t;
    b.bb.bounds(:,2) = b.bb.bounds(:,2) + t;
        
end

function b = poly_rotate(b, rot_mat)
 
    for i = 1:length(b.faces)
       
        b.faces(i).v1 = ((b.faces(i).v1 - b.o)' * rot_mat)'+b.o;
        b.faces(i).v2 = ((b.faces(i).v2 - b.o)' * rot_mat)'+b.o;
        b.faces(i).v3 = ((b.faces(i).v3 - b.o)' * rot_mat)'+b.o;
        
    end
    
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

