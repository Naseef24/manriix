import time
from datetime import datetime, timedelta
from manriix_camera.src.logger import logger as log


class SessionTracker:
    """Tracks session statistics including photos, videos, and timing"""
    
    def __init__(self):
        self.session_start_time = None
        self.session_end_time = None
        self.photo_count = 0
        self.video_count = 0
        self.successful_photos = 0
        self.successful_videos = 0
        self.failed_photos = 0
        self.failed_videos = 0
        self.total_video_duration = 0  # in seconds
        self.first_photo_time = None
        self.last_photo_time = None
        self.first_video_time = None
        self.last_video_time = None
        
    def start_session(self):
        """Start tracking a new session"""
        self.session_start_time = datetime.now()
        log.info(f"Session started at {self.session_start_time.strftime('%Y-%m-%d %H:%M:%S')}")
        
    def end_session(self):
        """End the current session"""
        self.session_end_time = datetime.now()
        log.info(f"Session ended at {self.session_end_time.strftime('%Y-%m-%d %H:%M:%S')}")
        
    def increment_photo_attempt(self):
        """Increment photo attempt counter"""
        self.photo_count += 1
        current_time = datetime.now()
        if self.first_photo_time is None:
            self.first_photo_time = current_time
        self.last_photo_time = current_time
        
    def increment_successful_photo(self):
        """Increment successful photo counter"""
        self.successful_photos += 1
        
    def increment_failed_photo(self):
        """Increment failed photo counter"""
        self.failed_photos += 1
        
    def increment_video_attempt(self, duration_seconds=0):
        """Increment video attempt counter"""
        self.video_count += 1
        self.total_video_duration += duration_seconds
        current_time = datetime.now()
        if self.first_video_time is None:
            self.first_video_time = current_time
        self.last_video_time = current_time
        
    def increment_successful_video(self):
        """Increment successful video counter"""
        self.successful_videos += 1
        
    def increment_failed_video(self):
        """Increment failed video counter"""
        self.failed_videos += 1
        
    def get_session_duration(self):
        """Get session duration in seconds"""
        if self.session_start_time is None:
            return 0
        end_time = self.session_end_time or datetime.now()
        return (end_time - self.session_start_time).total_seconds()
        
    def get_active_duration(self):
        """Get active capture duration (from first to last capture)"""
        if self.first_photo_time is None and self.first_video_time is None:
            return 0
            
        start_time = None
        end_time = None
        
        # Find earliest start time
        if self.first_photo_time and self.first_video_time:
            start_time = min(self.first_photo_time, self.first_video_time)
        elif self.first_photo_time:
            start_time = self.first_photo_time
        elif self.first_video_time:
            start_time = self.first_video_time
            
        # Find latest end time
        if self.last_photo_time and self.last_video_time:
            end_time = max(self.last_photo_time, self.last_video_time)
        elif self.last_photo_time:
            end_time = self.last_photo_time
        elif self.last_video_time:
            end_time = self.last_video_time
            
        if start_time and end_time:
            return (end_time - start_time).total_seconds()
        return 0
        
    def get_photos_per_minute(self):
        """Calculate photos per minute based on active duration"""
        active_duration = self.get_active_duration()
        if active_duration > 0 and self.successful_photos > 0:
            return (self.successful_photos / active_duration) * 60
        return 0
        
    def get_videos_per_minute(self):
        """Calculate videos per minute based on active duration"""
        active_duration = self.get_active_duration()
        if active_duration > 0 and self.successful_videos > 0:
            return (self.successful_videos / active_duration) * 60
        return 0
        
    def get_success_rate_photos(self):
        """Calculate photo success rate as percentage"""
        if self.photo_count > 0:
            return (self.successful_photos / self.photo_count) * 100
        return 0
        
    def get_success_rate_videos(self):
        """Calculate video success rate as percentage"""
        if self.video_count > 0:
            return (self.successful_videos / self.video_count) * 100
        return 0
        
    def format_duration(self, seconds):
        """Format duration in seconds to human readable format"""
        if seconds < 60:
            return f"{seconds:.1f} seconds"
        elif seconds < 3600:
            minutes = seconds / 60
            return f"{minutes:.1f} minutes"
        else:
            hours = seconds / 3600
            return f"{hours:.1f} hours"
            
    def print_session_summary(self):
        """Print comprehensive session summary"""
        if self.session_start_time is None:
            print("No session data available.")
            return
            
        self.end_session()
        
        print("\n" + "="*80)
        print("📸 CAMERA CONTROL SYSTEM - SESSION SUMMARY")
        print("="*80)
        
        # Session timing
        session_duration = self.get_session_duration()
        active_duration = self.get_active_duration()
        
        print(f"\n⏰ SESSION TIMING:")
        print(f"   Started:          {self.session_start_time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"   Ended:            {self.session_end_time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"   Total Duration:   {self.format_duration(session_duration)}")
        print(f"   Active Duration:  {self.format_duration(active_duration)}")
        
        # Photo statistics
        print(f"\n📷 PHOTO STATISTICS:")
        print(f"   Total Attempts:   {self.photo_count}")
        print(f"   Successful:       {self.successful_photos}")
        print(f"   Failed:           {self.failed_photos}")
        print(f"   Success Rate:     {self.get_success_rate_photos():.1f}%")
        if self.successful_photos > 0:
            print(f"   Photos/Minute:    {self.get_photos_per_minute():.2f}")
            
        # Video statistics
        print(f"\n🎥 VIDEO STATISTICS:")
        print(f"   Total Attempts:   {self.video_count}")
        print(f"   Successful:       {self.successful_videos}")
        print(f"   Failed:           {self.failed_videos}")
        print(f"   Success Rate:     {self.get_success_rate_videos():.1f}%")
        print(f"   Total Duration:   {self.format_duration(self.total_video_duration)}")
        if self.successful_videos > 0:
            print(f"   Videos/Minute:    {self.get_videos_per_minute():.2f}")
            print(f"   Avg Video Length: {self.format_duration(self.total_video_duration / self.successful_videos)}")
            
        # Overall statistics
        total_captures = self.successful_photos + self.successful_videos
        total_attempts = self.photo_count + self.video_count
        
        print(f"\n📊 OVERALL STATISTICS:")
        print(f"   Total Captures:   {total_captures}")
        print(f"   Total Attempts:   {total_attempts}")
        if total_attempts > 0:
            overall_success = (total_captures / total_attempts) * 100
            print(f"   Overall Success:  {overall_success:.1f}%")
        if active_duration > 0:
            captures_per_minute = (total_captures / active_duration) * 60
            print(f"   Captures/Minute:  {captures_per_minute:.2f}")
            
        # Activity timeline
        if self.first_photo_time or self.first_video_time:
            print(f"\n📅 ACTIVITY TIMELINE:")
            if self.first_photo_time:
                print(f"   First Photo:      {self.first_photo_time.strftime('%H:%M:%S')}")
            if self.last_photo_time and self.first_photo_time != self.last_photo_time:
                print(f"   Last Photo:       {self.last_photo_time.strftime('%H:%M:%S')}")
            if self.first_video_time:
                print(f"   First Video:      {self.first_video_time.strftime('%H:%M:%S')}")
            if self.last_video_time and self.first_video_time != self.last_video_time:
                print(f"   Last Video:       {self.last_video_time.strftime('%H:%M:%S')}")
                
        print("\n" + "="*80)
        print("Thank you for using Camera Control System! 📸✨")
        print("="*80 + "\n")


# Global session tracker instance
session_tracker = SessionTracker()