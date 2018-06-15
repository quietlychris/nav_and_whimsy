#[cfg(test)]
mod tests {

    use kinematics_lib::*;

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
        let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0));

        let mut desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        assert_eq!(current.determine_quadrant(desired),33);

        let mut desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        assert_eq!(current.determine_quadrant(desired),1);

        let mut desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        assert_eq!(current.determine_quadrant(desired),2);

        let mut desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        assert_eq!(current.determine_quadrant(desired),44);

        let mut desired = State::new(DoF::new(-1.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        assert_eq!(current.determine_quadrant(desired),3);

        let mut desired = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        assert_eq!(current.determine_quadrant(desired),22);

        let mut desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(-1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        assert_eq!(current.determine_quadrant(desired),4);
    }

    #[test]
    fn find_error_test() {
        assert_eq!(find_error(0.0,1.0),-1.0);
        assert_eq!(find_error(1.0,0.0),1.0);
        assert_eq!(find_error(1.0,2.0),-1.0);
        assert_eq!(find_error(2.0,1.0),1.0);
        assert_eq!(find_error(-1.0,1.0),-2.0);
        assert_eq!(find_error(1.0,1.0),0.0);
        assert_eq!(find_error(-1.0,-1.0),0.0);

    }

    #[test]
    fn calculate_yaw_test() {
        let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        let mut desired = State::new(DoF::new(1.0,0.0,0.0),
                                     DoF::new(1.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

        let quadrant = current.determine_quadrant(desired);
        let yaw = current.calculate_yaw(desired,quadrant);
        println!("Q1: quadrant: {} , then yaw: {}",quadrant,yaw);
        assert_eq!(yaw,45f32);

        desired = State::new(DoF::new(1.0,0.0,0.0),
                             DoF::new(-1.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0));

        let quadrant = current.determine_quadrant(desired);
        let yaw = current.calculate_yaw(desired,quadrant);
        println!("Q4: quadrant: {} , then yaw: {}",quadrant,yaw);
        assert_eq!(yaw,-45f32);

        desired = State::new(DoF::new(-1.0,0.0,0.0),
                             DoF::new(1.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0));

        let quadrant = current.determine_quadrant(desired);
        let yaw = current.calculate_yaw(desired,quadrant);
        println!("Q2: quadrant: {} , then yaw: {}",quadrant,yaw);
        assert_eq!(yaw,135f32);

        desired = State::new(DoF::new(-1.0,0.0,0.0),
                             DoF::new(-1.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0));

        let quadrant = current.determine_quadrant(desired);
        let yaw = current.calculate_yaw(desired,quadrant);
        println!("Q3: quadrant: {} , then yaw: {}",quadrant,yaw);
        assert_eq!(yaw,-135f32);

        desired = State::new(DoF::new(0.0,0.0,0.0),
                             DoF::new(1.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0));

        let quadrant = current.determine_quadrant(desired);
        let yaw = current.calculate_yaw(desired,quadrant);
        println!("North: quadrant: {} , then yaw: {}",quadrant,yaw);
        assert_eq!(yaw,90f32);

        desired = State::new(DoF::new(0.0,0.0,0.0),
                             DoF::new(-1.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0));

        let quadrant = current.determine_quadrant(desired);
        let yaw = current.calculate_yaw(desired,quadrant);
        println!("South: quadrant: {} , then yaw: {}",quadrant,yaw);
        assert_eq!(yaw,-90f32);

        desired = State::new(DoF::new(1.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0));

        let quadrant = current.determine_quadrant(desired);
        let yaw = current.calculate_yaw(desired,quadrant);
        println!("East: quadrant: {} , then yaw: {}",quadrant,yaw);
        assert_eq!(yaw,0f32);

        desired = State::new(DoF::new(-1.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0));

        let quadrant = current.determine_quadrant(desired);
        let yaw = current.calculate_yaw(desired,quadrant);
        println!("West: quadrant: {} , then yaw: {}",quadrant,yaw);
        assert_eq!(yaw,180f32);

    }
}
