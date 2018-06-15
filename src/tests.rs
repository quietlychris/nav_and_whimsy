#[cfg(test)]
mod tests {

    // Note: can put #[ignore] underneath #[test] to stop that test from running
    // Running with '$ cargo test -- --noignore' will send command-line output even if test passes

    use kinematics_lib::*;
    use std::f32;
    use DT;

    #[test]
    fn sign_check_test() {
        assert_eq!(true,sign_check(-2f32,-3f32));
        assert_eq!(false,sign_check(1f32,-1f32));
        assert_eq!(true,sign_check(0f32,-3f32));
        assert_eq!(true,sign_check(0f32,3f32));
        assert_eq!(true,sign_check(-2f32,0f32));
        assert_eq!(true,sign_check(2f32,0f32));
    }

    #[test]
    fn determine_quadrant_test() {


        let current = State::new(DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0));

        // Quadrant 1
        let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        assert_eq!(current.determine_quadrant(desired),"q1");

        // Quadrant 2
        let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        assert_eq!(current.determine_quadrant(desired),"q2");

        // Quadrant 3
        let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        assert_eq!(current.determine_quadrant(desired),"q3");

        // Quadrant 4
        let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        assert_eq!(current.determine_quadrant(desired),"q4");

        // North
        let desired = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        assert_eq!(current.determine_quadrant(desired),"north");

        // South
        let desired = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        assert_eq!(current.determine_quadrant(desired),"south");

        // East
        let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        assert_eq!(current.determine_quadrant(desired),"east");

        // West
        let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        assert_eq!(current.determine_quadrant(desired),"west");

    }

    #[test]
    fn calculate_yaw_test() {
        let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        // Quadrant 1, +45 degrees
        let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        let desired_yaw = current.calculate_yaw(desired);
        assert_eq!(desired_yaw,45f32);

        // Quadrant 2, +135 degrees
        let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        let desired_yaw = current.calculate_yaw(desired);
        assert_eq!(desired_yaw,135f32);

        // Quadrant 3, -135 degrees
        let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        let desired_yaw = current.calculate_yaw(desired);
        assert_eq!(desired_yaw,-135f32);

        // Quadrant 4, -45 degrees
        let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        let desired_yaw = current.calculate_yaw(desired);
        assert_eq!(desired_yaw,-45f32);

        // North, +90 degrees
        let desired = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        let desired_yaw = current.calculate_yaw(desired);
        assert_eq!(desired_yaw,90f32);

        // South, -90 degrees
        let desired = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        let desired_yaw = current.calculate_yaw(desired);
        assert_eq!(desired_yaw,-90f32);

        // East, 0 degrees
        let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        let desired_yaw = current.calculate_yaw(desired);
        assert_eq!(desired_yaw,0f32);

        // Quadrant 1, 180 degrees
        let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));
        let desired_yaw = current.calculate_yaw(desired);
        assert_eq!(desired_yaw,180f32);








    }

    #[test]
    fn slow_down_test() { // Has determine_quadrant(), calculate_yaw(), update_position(), and update_direction() dependencies
        let yaw_allowance: f32 = 3.0;

        { // Slowing down in quadrant 1
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.00,0.0),
                                         DoF::new(0.0,0.0,0.0));

            let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let position_smd = SMD::new(0.5,0.9,0.20);
            let yaw_smd = SMD::new(0.1,0.2,1.0);

            let mut time: f32 = 0.0;
            while time < 100.0f32
            {
                if time < 8.0f32 {
                    let desired_yaw = current.calculate_yaw(desired);
                    current.update_direction(desired_yaw,yaw_smd);
                    current.update_position(desired,position_smd,yaw_allowance);
                }
                else
                {
                    current.trend_speed_towards_zero(yaw_smd);
                }
                time = time + DT;
            }
            let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
            //println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            if radius < 2f32.powf(0.5) { println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius); }
            assert_eq!(true,radius<2f32.powf(0.5));
        }

        { // Slowing down in quadrant 2
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.00,0.0),
                                         DoF::new(0.0,0.0,0.0));

            let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let position_smd = SMD::new(0.5,0.9,0.20);
            let yaw_smd = SMD::new(0.1,0.2,1.0);

            let mut time: f32 = 0.0;
            while time < 100.0f32
            {
                if time < 8.0f32 {
                    let desired_yaw = current.calculate_yaw(desired);
                    current.update_direction(desired_yaw,yaw_smd);
                    current.update_position(desired,position_smd,yaw_allowance);
                }
                else
                {
                    current.trend_speed_towards_zero(yaw_smd);
                }
                time = time + DT;
            }
            let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
            //println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            if radius < 2f32.powf(0.5) { println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius); }
            assert_eq!(true,radius<2f32.powf(0.5));
        }

        { // Slowing down in quadrant 3
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.00,0.0),
                                         DoF::new(0.0,0.0,0.0));

            let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let position_smd = SMD::new(0.5,0.9,0.20);
            let yaw_smd = SMD::new(0.1,0.2,1.0);

            let mut time: f32 = 0.0;
            while time < 100.0f32
            {
                if time < 8.0f32 {
                    let desired_yaw = current.calculate_yaw(desired);
                    current.update_direction(desired_yaw,yaw_smd);
                    current.update_position(desired,position_smd,yaw_allowance);
                }
                else
                {
                    current.trend_speed_towards_zero(yaw_smd);
                }
                time = time + DT;
            }
            let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
            //println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            if radius < 2f32.powf(0.5) { println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius); }
            assert_eq!(true,radius<2f32.powf(0.5));
        }

        { // Slowing down in quadrant 4
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.00,0.0),
                                         DoF::new(0.0,0.0,0.0));

            let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let position_smd = SMD::new(0.5,0.9,0.20);
            let yaw_smd = SMD::new(0.1,0.2,1.0);

            let mut time: f32 = 0.0;
            while time < 100.0f32
            {
                if time < 8.0f32 {
                    let desired_yaw = current.calculate_yaw(desired);
                    current.update_direction(desired_yaw,yaw_smd);
                    current.update_position(desired,position_smd,yaw_allowance);
                }
                else
                {
                    current.trend_speed_towards_zero(yaw_smd);
                }
                time = time + DT;
            }
            let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
            //println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            if radius < 2f32.powf(0.5) { println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius); }
            assert_eq!(true,radius<2f32.powf(0.5));
        }

        { // Slowing directly North
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.00,0.0),
                                         DoF::new(0.0,0.0,0.0));

            let desired = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let position_smd = SMD::new(0.5,0.9,0.20);
            let yaw_smd = SMD::new(0.1,0.2,1.0);

            let mut time: f32 = 0.0;
            while time < 100.0f32
            {
                if time < 8.0f32 {
                    let desired_yaw = current.calculate_yaw(desired);
                    current.update_direction(desired_yaw,yaw_smd);
                    current.update_position(desired,position_smd,yaw_allowance);
                }
                else
                {
                    current.trend_speed_towards_zero(yaw_smd);
                }
                time = time + DT;
                let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
                println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            }
            let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
            //println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            if radius < 2f32.powf(0.5) { println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius); }
            assert_eq!(true,radius<2f32.powf(0.5));
        }

        { // Slowing directly South
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.00,0.0),
                                         DoF::new(0.0,0.0,0.0));

            let desired = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let position_smd = SMD::new(0.5,0.9,0.20);
            let yaw_smd = SMD::new(0.1,0.2,1.0);

            let mut time: f32 = 0.0;
            while time < 100.0f32
            {
                if time < 8.0f32 {
                    let desired_yaw = current.calculate_yaw(desired);
                    current.update_direction(desired_yaw,yaw_smd);
                    current.update_position(desired,position_smd,yaw_allowance);
                }
                else
                {
                    current.trend_speed_towards_zero(yaw_smd);
                }
                time = time + DT;
                let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
                println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            }
            let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
            //println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            if radius < 2f32.powf(0.5) { println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius); }
            assert_eq!(true,radius<2f32.powf(0.5));
        }

        { // Slowing directly East
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.00,0.0),
                                         DoF::new(0.0,0.0,0.0));

            let desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let position_smd = SMD::new(0.5,0.9,0.20);
            let yaw_smd = SMD::new(0.1,0.2,1.0);

            let mut time: f32 = 0.0;
            while time < 100.0f32
            {
                if time < 8.0f32 {
                    let desired_yaw = current.calculate_yaw(desired);
                    current.update_direction(desired_yaw,yaw_smd);
                    current.update_position(desired,position_smd,yaw_allowance);
                }
                else
                {
                    current.trend_speed_towards_zero(yaw_smd);
                }
                time = time + DT;
                let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
                println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            }
            let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
            //println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            if radius < 2f32.powf(0.5) { println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius); }
            assert_eq!(true,radius<2f32.powf(0.5));
        }

        { // Slowing directly West
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.00,0.0),
                                         DoF::new(0.0,0.0,0.0));

            let desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let position_smd = SMD::new(0.5,0.9,0.20);
            let yaw_smd = SMD::new(0.1,0.2,1.0);

            let mut time: f32 = 0.0;
            while time < 100.0f32
            {
                if time < 8.0f32 {
                    let desired_yaw = current.calculate_yaw(desired);
                    current.update_direction(desired_yaw,yaw_smd);
                    current.update_position(desired,position_smd,yaw_allowance);
                }
                else
                {
                    current.trend_speed_towards_zero(yaw_smd);
                }
                time = time + DT;
                let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
                println!("yaw: {:.2}  [{:.2},{:.2}] => radius = {:.2}",current.yaw.var,current.xaxis.var,current.yaxis.var,radius);
            }
            let radius = (current.xaxis.var.powf(2.0) + current.yaxis.var.powf(2.0)).powf(0.5);
            //println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius);
            if radius < 2f32.powf(0.5) { println!("[{:.2},{:.2}] => radius = {:.2}",current.xaxis.var,current.yaxis.var,radius); }
            assert_eq!(true,radius<2f32.powf(0.5));
        }




    }

    #[test]
    fn kinematics_test() {
        let limit: f32 = 0.1;
        let mut dir_vector: Vec<(f32,f32)> = Vec::new();
        for y in -1isize..2isize {
            for x in -1isize..2isize {
                let mut dir = (x as f32,y as f32);
                dir_vector.push(dir);
            }
        }

        for a in 0..dir_vector.len()
        { // Travel along the positive x-axis
            let mut time: f32 = 0.0;
            let time_limit: f32 = 10.0;
            let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.0,0.0),
                                         DoF::new(0.0,0.0,0.0));
            let xvardd = dir_vector[a].0;
            let yvardd = dir_vector[a].1;
            println!("x-y acceleration: [{},{}]",xvardd,yvardd);
            let answer_x = current.xaxis.var + current.xaxis.vard*time_limit + 0.5*xvardd*(time_limit.powf(2.0));
            let answer_y = current.yaxis.var + current.yaxis.vard*time_limit + 0.5*yvardd*(time_limit.powf(2.0));
            while time < time_limit {
                // Based on the common kinematic equation x = x_o + v*t + .5*a*t^2
                current.kinematic_update(xvardd,yvardd,0.0);
                time = time + DT;
            }
            println!("[Correct answer, result] -> x:[{:.2},{:.2}] y:[{:.2},{:.2}]",answer_x,current.xaxis.var,answer_y,current.yaxis.var);
            let tof = if ( (current.xaxis.var - answer_x).abs() <= limit && (current.yaxis.var - answer_y).abs() <= limit ) {true} else {false};
            assert_eq!(true,tof);
        }





    }
}
