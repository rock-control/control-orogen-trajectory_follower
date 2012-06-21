#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'vizkit'

include Orocos

BASE_DIR = File.expand_path('..', File.dirname(__FILE__))
ENV['PKG_CONFIG_PATH'] = "#{BASE_DIR}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize
Orocos.conf.load_dir("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/asguard/config/orogen")

Orocos.run 'test_trajectory' do |p|
    Orocos.log_all_ports 
    tf = p.task 'trajectory'
    Orocos.conf.apply(trajectory_follower, ['default'])

    if ARGV.size == 0 then
	log_replay = Orocos::Log::Replay.open( "corridor_following.0.log", "eslam_pose_estimator.0.log" ) 
    else
	log_replay = Orocos::Log::Replay.open( ARGV[0]+"corridor_following.0.log" , ARGV[0]+"eslam_pose_estimator.0.log" ) 
    end
    log_replay.corridor_following.trajectory.connect_to( tf.trajectory, :type => :buffer, :size => 1 )
    log_replay.eslam_pose_estimator.pose_samples.connect_to( tf.pose, :type => :buffer, :size => 1 )
    
    tf.configure
    tf.start

    log_replay.run
end

