from autonomous.auto_modes import get_next_auto_command

def test_get_next_auto_command():
    for i in get_next_auto_command("1 amp + 3_speakers"):
        print(i)