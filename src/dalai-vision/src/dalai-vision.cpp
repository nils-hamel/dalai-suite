/*
 *  dalai-suite - vision
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2018 DHLAB, EPFL
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

    # include "dalai-vision.hpp"

/*
    source - constructor/destructor methods
 */

    dl_vision_t::dl_vision_t()

        : vs_window( NULL )
        , vs_context( 0 )

        , vs_width( 0 )
        , vs_height( 0 )
        , vs_near( 0.001 )
        , vs_far( 100.0 )

        , vs_state( true )

        , vs_angle_x( 0.0 )
        , vs_angle_y( 0.0 )
        , vs_angle_z( 0.0 )
        , vs_trans_x( 0.0 )
        , vs_trans_y( 0.0 )
        , vs_trans_z( 0.0 )

        , vs_mouse_x( 0 )
        , vs_mouse_y( 0 )

    {

        /* diplay mode variables */
        SDL_DisplayMode dl_display;

        /* initialise sdl */
        if ( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {

            /* throw exception */
            throw( LC_ERROR_CONTEXT );

        }

        /* retrieve current display mode */
        if ( SDL_GetCurrentDisplayMode( 0, & dl_display ) < 0 ) {

            /* throw exception */
            throw( LC_ERROR_CONTEXT );

        }

        /* retreive display resolution */
        vs_width  = dl_display.w;
        vs_height = dl_display.h;

        /* enable double-buffering */
        SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );

        /* define frame buffer components size */
        SDL_GL_SetAttribute( SDL_GL_RED_SIZE  , 8 );
        SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE, 8 );
        SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE , 8 );
        SDL_GL_SetAttribute( SDL_GL_ALPHA_SIZE, 8 );

        /* define depth buffer size */
        SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 24 );

        /* create sdl window */
        if ( ( vs_window = SDL_CreateWindow( "", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, vs_width, vs_height, SDL_WINDOW_FULLSCREEN | SDL_WINDOW_OPENGL ) ) == NULL ) {

            /* throw exception */
            throw( LC_ERROR_CONTEXT );

        }

        /* create opengl context */
        if ( ( vs_context = SDL_GL_CreateContext( vs_window ) ) == NULL ) {

            /* throw exception */
            throw( LC_ERROR_CONTEXT );

        }

        /* opengl clear color */
        glClearColor( 0.00, 0.02, 0.05, 0.0 );

        /* opengl clear depth */
        glClearDepth( 1.0 );

        /* opengl states */
        glEnable( GL_DEPTH_TEST );

        /* opengl blending function */
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    }

    dl_vision_t::~dl_vision_t() {

        /* delete opengl context */
        SDL_GL_DeleteContext( vs_context );

        /* delete window */
        SDL_DestroyWindow( vs_window );

        /* terminate sdl */
        SDL_Quit();

    }

/*
    source - accessor methods
 */

    bool dl_vision_t::vs_get_click( int const dl_mx, int const dl_my, double * const dl_px, double * const dl_py, double * const dl_pz ) {

        /* click depth variables */
        float dl_depth( 0.0 );

        /* retrieve click depth */
        glReadPixels( dl_mx, vs_height - dl_my, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, & dl_depth );

        /* check depth value */
        if ( dl_depth < 1.0 ) {

            /* retrieve model click positon */
            gluUnProject( dl_mx, vs_height - dl_my, dl_depth, vs_modelview, vs_projection, vs_viewport, dl_px, dl_py, dl_pz );

            /* return status */
            return( true );

        /* return status */
        } else { return( false ); }

    }

/*
    source - mutator methods
 */

    void dl_vision_t::vs_set_projection( dl_model_t & dl_model ) {

        /* opengl viewport */
        glViewport( 0.0, 0.0, vs_width, vs_height );

        /* push viewport vector */
        glGetIntegerv( GL_VIEWPORT, vs_viewport );

        /* opengl matrix mode */
        glMatrixMode( GL_PROJECTION );

        /* reset matrix coefficients */
        glLoadIdentity();

        /* compute matrix coefficients */
        //gluPerspective( 45.0, double( vs_width ) / double( vs_height ), dl_model.ml_get_mdmv() * 10.0, dl_model.ml_get_span() * 2.0 );
        gluPerspective( 45.0, double( vs_width ) / double( vs_height ), dl_model.ml_get_span() * 0.01, dl_model.ml_get_span() * 10.0 );

        /* push projection matrix */
        glGetDoublev( GL_PROJECTION_MATRIX, vs_projection );

    }

    void dl_vision_t::vs_set_viewpoint( dl_model_t & dl_model ) {

        /* reset rotation */
        vs_angle_x = 0.0;
        vs_angle_y = 0.0;
        vs_angle_z = 0.0;

        /* reset translation */
        vs_trans_x = 0.0;
        vs_trans_y = 0.0;
        vs_trans_z = -dl_model.ml_get_span();

    }

/*
    source - execution methods
 */

    void dl_vision_t::vs_execution( dl_model_t & dl_model ) {

        /* principale execution loop */
        while ( vs_state == true ) {

            /* events management */
            while ( SDL_PollEvent( & dl_event ) > 0 ) {

                /* switch on event type */
                switch ( dl_event.type ) {

                    /* event : keydown */
                    case ( SDL_KEYDOWN ) : {

                        /* specialised method */
                        vs_keydown( dl_event.key, dl_model );

                    } break;

                    /* event : mouse button */
                    case ( SDL_MOUSEBUTTONDOWN ) : {

                        /* specialised method */
                        vs_button( dl_event.button, dl_model );

                    } break;

                    /* event : mouse motion */
                    case ( SDL_MOUSEMOTION ) : {

                        /* specialised method */
                        vs_motion( dl_event.motion, dl_model );

                    } break;

                    /* event : mouse wheele */
                    case ( SDL_MOUSEWHEEL ) : {

                        /* specialised method */
                        vs_wheel( dl_event.wheel, dl_model );

                    } break;

                };

            }

            /* opengl buffer clear */
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

            /* opengl matrix mode */
            glMatrixMode( GL_MODELVIEW );

            /* reset matrix coefficients */
            glLoadIdentity();

            /* push matrix */
            glPushMatrix();

                /* view translations */
                glTranslatef( vs_trans_x, vs_trans_y, vs_trans_z );

                /* view rotations */
                glRotatef( vs_angle_x, 1.0, 0.0, 0.0 );
                glRotatef( vs_angle_y, 0.0, 1.0, 0.0 );
                glRotatef( vs_angle_z, 0.0, 0.0, 1.0 );

                /* push matrix */
                glPushMatrix();

                    /* display frame */
                    dl_model.ml_ren_frame();

                /* pop matrix */
                glPopMatrix();

                /* push matrix */
                glPushMatrix();

                    /* push matrix */
                    glPushMatrix();

                        /* diplay model */
                        dl_model.ml_ren_model();

                        /* retrieve modelview matrix */
                        glGetDoublev( GL_MODELVIEW_MATRIX, vs_modelview );

                    /* pop matrix */
                    glPopMatrix();

                    /* push matrix */
                    glPushMatrix();

                        /* display surface */
                        dl_model.ml_ren_surface();

                    /* pop matrix */
                    glPopMatrix();

                /* pop matrix */
                glPopMatrix();

            /* pop matrix */
            glPopMatrix();

            /* swap buffers */
            SDL_GL_SwapWindow( vs_window );

        }

    }

/*
    source - event methods
 */

    void dl_vision_t::vs_keydown( SDL_KeyboardEvent dl_event, dl_model_t & dl_model ) {

        /* switch on keycode */
        switch ( dl_event.keysym.sym ) {

            /* keycode : [escape] */
            case ( SDLK_ESCAPE ) : {

                /* update state */
                vs_state = false;

            } break;

            case ( SDLK_1 ) :
            case ( SDLK_2 ) :
            case ( SDLK_3 ) :
            case ( SDLK_4 ) : {

                /* update model point size */
                dl_model.ml_set_pointsize( dl_event.keysym.sym - SDLK_1 + 1 );

            } break;

            case ( SDLK_r ) : {

                /* reset viewpoint */
                vs_set_viewpoint( dl_model );

            } break;

            case ( SDLK_TAB ) : {

                /* switch display flag */
                dl_model.ml_set_surface_switch();

            } break;

            case ( SDLK_RETURN ) : {

                /* compute and display intersection */
                dl_model.ml_get_intersect();

            } break;

            case ( SDLK_BACKSPACE ) : {

                /* clear points */
                dl_model.ml_set_point_clear();

            } break;

            case ( SDLK_q ) : {

                /* select active surface */
                dl_model.ml_set_surface( 0 );

            } break;

            case ( SDLK_w ) : {

                /* select active surface */
                dl_model.ml_set_surface( 1 );

            } break;

            case ( SDLK_e ) : {

                /* select active surface */
                dl_model.ml_set_surface( 2 );

            } break;

            case ( SDLK_a ) : {

                /* surface point auto-push */
                dl_model.ml_set_point_auto();

            } break;

        };

    }

    void dl_vision_t::vs_button( SDL_MouseButtonEvent dl_event, dl_model_t & dl_model ) {

        /* click components variables */
        double dl_mx( 0.0 );
        double dl_my( 0.0 );
        double dl_mz( 0.0 );

        /* push mouse position at click */
        vs_mouse_x = dl_event.x;
        vs_mouse_y = dl_event.y;

        /* switch on button */
        if ( dl_event.button == SDL_BUTTON_LEFT ) {

            /* switch on click type */
            if ( dl_event.clicks == 2 ) {

                /* compute and check click components */
                if ( vs_get_click( dl_event.x, dl_event.y, & dl_mx, & dl_my, & dl_mz ) == true ) {

                    /* update model center */
                    dl_model.ml_set_center( dl_mx, dl_my, dl_mz );

                }

            }

        } else if ( dl_event.button == SDL_BUTTON_MIDDLE ) {

            /* switch on click type */
            if ( dl_event.clicks == 1 ) {

                /* push model center to surface */
                dl_model.ml_set_point_push();

            }

        }

    }

    void dl_vision_t::vs_motion( SDL_MouseMotionEvent dl_event, dl_model_t & dl_model ) {

        /* inertia variables */
        float dl_inertia( 1.0 );

        /* analyse keyboard modifiers */
        if ( ( SDL_GetModState() & KMOD_CTRL  ) != 0 ) dl_inertia *= 5;
        if ( ( SDL_GetModState() & KMOD_SHIFT ) != 0 ) dl_inertia /= 5;

        /* switch on button */
        if ( ( dl_event.state & SDL_BUTTON_LMASK ) != 0 ) {

            /* update inertia value */
            dl_inertia *= DL_INERTIA_ANGLE;

            /* update view angle */
            vs_angle_x += dl_inertia * float( int( dl_event.y ) - vs_mouse_y );
            vs_angle_z += dl_inertia * float( int( dl_event.x ) - vs_mouse_x ) * ( vs_modelview[10] < 0 ? -1.0 : +1.0 );

        } else if ( ( dl_event.state & SDL_BUTTON_RMASK ) != 0 ) {

            /* update inertia value */
            dl_inertia *= DL_INERTIA_TRANS * dl_model.ml_get_mdmv();

            /* update model translation */
            vs_trans_x += dl_inertia * DL_INERTIA_TRANS * float( int( dl_event.x ) - vs_mouse_x );
            vs_trans_y -= dl_inertia * DL_INERTIA_TRANS * float( int( dl_event.y ) - vs_mouse_y );

        }

    }

    void dl_vision_t::vs_wheel( SDL_MouseWheelEvent dl_event, dl_model_t & dl_model ) {

        /* inertia variables */
        float dl_inertia( dl_model.ml_get_mdmv() * DL_INERTIA_WZOOM );

        /* analyse keyboard modifiers */
        if ( ( SDL_GetModState() & KMOD_CTRL  ) != 0 ) dl_inertia *= 5;
        if ( ( SDL_GetModState() & KMOD_SHIFT ) != 0 ) dl_inertia /= 5;

        /* switch on wheel direction */
        if ( dl_event.y > 0 ) {

            /* update model translation */
            vs_trans_z += dl_inertia;

        } else if ( dl_event.y < 0 ) {

            /* update model translation */
            vs_trans_z -= dl_inertia;

        }

    }

/*
    source - main function
 */

    int main( int argc, char ** argv ) {

    /* error management */
    try {

        /* vision class variables */
        dl_vision_t dl_vision;

        /* model class variables */
        dl_model_t dl_model( lc_read_string( argc, argv, "--model", "-m" ) );

        /* set projection matrix */
        dl_vision.vs_set_projection( dl_model );

        /* set initial viewpoint */
        dl_vision.vs_set_viewpoint( dl_model );

        /* principale execution loop */
        dl_vision.vs_execution( dl_model );

    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

