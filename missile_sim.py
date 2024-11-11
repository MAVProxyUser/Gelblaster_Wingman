def apply_centering(offset_x, offset_y):
    # Apply the necessary adjustments to center the view
    # This could involve moving a camera, adjusting the viewport, etc.
    # Example:
    adjust_pan_tilt(offset_x, offset_y) 

def handle_events(game_state):
    for event in pygame.event.get():
        # Existing event handling...
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = pygame.mouse.get_pos()
            
            # Check if Auto button is clicked in Live mode
            if game_state.display_mode == "Live":
                if auto_button_rect.collidepoint(mouse_pos):
                    # Toggle auto-tracking mode
                    game_state.auto_mode = not game_state.auto_mode