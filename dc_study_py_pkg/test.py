import time
import os

import time
import os

def test_break():
    # Clear the terminal for better visibility
    os.system('cls' if os.name == 'nt' else 'clear')

    # Display the break message in a more visual style
    print("\n" + "="*40)
    print(" " * 10 + "\033[1;32m Time for a Break! \033[0m")
    print("="*40)
    
    # Big cat ASCII art for a comforting break reminder
    print("""
        /\_/\  
       ( o.o )   Meow~ It's break time!
        > ^ <
    """)
    
    print("\033[1;36mTake a moment to relax and stretch.\033[0m")
    
    # Countdown for a 5-minute break
    total_seconds = 300  # 5 minutes
    for i in range(total_seconds, 0, -1):
        mins, secs = divmod(i, 60)
        time_display = f"{mins:02}:{secs:02}"
        print(f"\r\033[1;33mReturning in {time_display} (mm:ss)...\033[0m", end="")
        time.sleep(1)

    # Clear break message
    os.system('cls' if os.name == 'nt' else 'clear')
    print("Break over! Let's get back to work.")
def main():
    test_break()
if __name__ == main():
    main()