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

    }


}
